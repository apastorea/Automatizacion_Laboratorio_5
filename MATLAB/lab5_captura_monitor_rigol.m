function outputFolder = lab5_captura_monitor_rigol(label, forceDelay, timeScale, ch1Scale, ch2Scale, expectedMode)
%LAB5_CAPTURA_MONITOR_RIGOL Captura sincronizada del montaje del Lab. 5.
%   CH1 se interpreta como J5 (mando fisico u) y CH2 como J7 (salida y).
%   El PSoC debe estar programado con el modo indicado. La funcion arma el
%   RIGOL, espera un reinicio del PSoC por COM4 y guarda pantalla, trazas,
%   telemetria y estadisticas de offset. No aplica ningun mando a la planta.

arguments
    label (1,1) string = "monitor_offset"
    forceDelay (1,1) double {mustBePositive} = 3.20
    timeScale (1,1) double {mustBePositive} = 0.050
    ch1Scale (1,1) double {mustBePositive} = 0.050
    ch2Scale (1,1) double {mustBePositive} = 0.050
    expectedMode (1,1) string {mustBeMember(expectedMode,["MONITOR","CALIBRATION","CONTROL"])} = "MONITOR"
end

scriptFolder = fileparts(mfilename("fullpath"));
dataRoot = fullfile(scriptFolder,"datos_experimentales");
stamp = string(datetime("now",Format="yyyyMMdd_HHmmss"));
outputFolder = fullfile(dataRoot,"lab5_" + label + "_" + stamp);
mkdir(outputFolder);

devices = visadevlist;
rigolRow = contains(devices.ResourceName,"0x1AB1",IgnoreCase=true) & ...
    contains(devices.SerialNumber,"DS1ZE275014864",IgnoreCase=true);
assert(nnz(rigolRow)==1,"No se encontro exactamente un RIGOL DS1202Z-E.");
resourceName = devices.ResourceName(rigolRow);

scope = visadev(resourceName);
scope.Timeout = 15;
scope.ByteOrder = "little-endian";
configureScope(scope,timeScale,ch1Scale,ch2Scale,expectedMode);

port = serialport("COM4",57600,Timeout=1);
configureTerminator(port,"LF");
flush(port);

fprintf("CAPTURA_LISTA: pulse RESET en el PSoC una sola vez.\n");
lines = strings(0,1);
started = false;
finished = false;
scopeForced = false;
triggerDecisionMade = false;
captureTimer = [];
deadline = tic;
while toc(deadline) < 3600
    if started && expectedMode~="MONITOR" && ~triggerDecisionMade && ...
            toc(captureTimer)>=forceDelay
        stateAtDecision = string(strip(writeread(scope,":TRIG:STAT?")));
        if stateAtDecision~="STOP"
            writeline(scope,":TFOR");
            scopeForced = true;
            fprintf("DISPARO_FORZADO: %.3f s despues del encabezado (%s).\n", ...
                forceDelay,expectedMode);
        end
        triggerDecisionMade = true;
    end
    if port.NumBytesAvailable==0
        pause(0.02);
        continue
    end
    rawLine = readline(port);
    if ~(isstring(rawLine) || ischar(rawLine))
        continue
    end
    line = strip(string(rawLine));
    if strlength(line)==0
        continue
    end
    lines(end+1,1) = line; %#ok<AGROW>
    if ~started && startsWith(line,"# LAB5")
        if ~contains(line,"mode=" + expectedMode)
            fprintf("ENCABEZADO_IGNORADO: %s\n",line);
            continue
        end
        started = true;
        captureTimer = tic;
        fprintf("CAPTURA_INICIADA: encabezado LAB5 %s detectado.\n",expectedMode);
        pause(0.25);
        writeline(scope,":SING");
        fprintf("OSCILOSCOPIO_ARMADO: despues del transitorio de RESET.\n");
        if expectedMode=="MONITOR"
            pause(forceDelay);
            writeline(scope,":TFOR");
            fprintf("DISPARO_FORZADO: %.3f s despues del encabezado.\n",forceDelay);
        else
            fprintf("ESPERANDO_DISPARO: flanco de CH1/J5 a 75 mV (%s).\n",expectedMode);
        end
    end
    if started && startsWith(line,"# END")
        finished = true;
        break
    end
end
assert(started,"No se detecto un nuevo encabezado LAB5 por COM4.");
assert(finished,"La telemetria del PSoC no termino antes del timeout.");

triggerState = string(strip(writeread(scope,":TRIG:STAT?")));
physicalTriggerDetected = triggerState=="STOP" && ~scopeForced;
if expectedMode~="MONITOR" && ~physicalTriggerDetected
    if triggerState~="STOP"
        warning("LAB5:PhysicalTriggerMissing", ...
            "El RIGOL no detecto el flanco fisico esperado en CH1/J5; se fuerza la captura para conservar UART y trazas.");
        writeline(scope,":TFOR");
        scopeForced = true;
        pause(0.20);
    end
elseif expectedMode=="MONITOR" && triggerState~="STOP"
    writeline(scope,":STOP");
end

writeTextLines(fullfile(outputFolder,"psoc_uart.txt"),lines);
uartTable = parseUartTable(lines);
writetable(uartTable,fullfile(outputFolder,"psoc_uart.csv"));
captureScreen(scope,fullfile(outputFolder,"pantalla.png"));
ch1 = captureWaveform(scope,"CHAN1");
ch2 = captureWaveform(scope,"CHAN2");
writetable(ch1,fullfile(outputFolder,"chan1_J5.csv"));
writetable(ch2,fullfile(outputFolder,"chan2_J7.csv"));

stats = table(...
    ["CH1";"CH2"], ["J5: mando fisico u(t)";"J7: salida de planta y(t)"], ...
    [mean(ch1.voltage_V,"omitmissing");mean(ch2.voltage_V,"omitmissing")], ...
    [std(ch1.voltage_V,0,"omitmissing");std(ch2.voltage_V,0,"omitmissing")], ...
    [min(ch1.voltage_V,[],"omitmissing");min(ch2.voltage_V,[],"omitmissing")], ...
    [max(ch1.voltage_V,[],"omitmissing");max(ch2.voltage_V,[],"omitmissing")], ...
    VariableNames=["channel","location","mean_V","std_V","min_V","max_V"]);
writetable(stats,fullfile(outputFolder,"resumen_offset.csv"));

metadata = struct;
metadata.capturedAt = string(datetime("now",TimeZone="local", ...
    Format="yyyy-MM-dd'T'HH:mm:ssXXX"));
metadata.resource = resourceName;
metadata.idn = strtrim(writeread(scope,"*IDN?"));
metadata.label = label;
metadata.mode = expectedMode;
metadata.forceDelay_s = forceDelay;
metadata.triggerState = triggerState;
metadata.physicalTriggerDetected = physicalTriggerDetected;
metadata.triggerWasForced = scopeForced;
metadata.timeScale_s_div = timeScale;
metadata.ch1Scale_V_div = ch1Scale;
metadata.ch2Scale_V_div = ch2Scale;
metadata.channelMap = struct("CH1","J5: mando fisico u(t)", ...
    "CH2","J7: salida de planta y(t)");
fid = fopen(fullfile(outputFolder,"metadata.json"),"w");
assert(fid~=-1,"No se pudo crear metadata.json.");
cleanupFile = onCleanup(@() fclose(fid));
fprintf(fid,"%s",jsonencode(metadata,PrettyPrint=true));

disp(stats)
fprintf("CAPTURA_COMPLETA: %s\n",outputFolder);
end

function configureScope(scope,timeScale,ch1Scale,ch2Scale,expectedMode)
if expectedMode~="MONITOR"
    triggerSource = "CHAN1";
    % Se arma el osciloscopio despues del encabezado de arranque. El nivel
    % queda por encima del mando estacionario y por debajo del primer pico,
    % de modo que el disparo corresponda al escalon y no al RESET.
    triggerLevel = 0.075;
else
    triggerSource = "CHAN2";
    triggerLevel = 0;
end
commands = [
    ":STOP"
    ":CHAN1:DISP ON"
    ":CHAN2:DISP ON"
    ":CHAN1:COUP DC"
    ":CHAN2:COUP DC"
    ":CHAN1:BWL 20M"
    ":CHAN2:BWL 20M"
    ":CHAN1:PROB 10"
    ":CHAN2:PROB 10"
    ":CHAN1:SCAL " + compose("%.9g",ch1Scale)
    ":CHAN2:SCAL " + compose("%.9g",ch2Scale)
    ":CHAN1:OFFS 0"
    ":CHAN2:OFFS 0"
    ":TIM:MODE MAIN"
    ":TIM:SCAL " + compose("%.9g",timeScale)
    ":TIM:OFFS 0"
    ":ACQ:TYPE HRES"
    ":TRIG:MODE EDGE"
    ":TRIG:EDGE:SOUR " + triggerSource
    ":TRIG:EDGE:SLOP POS"
    ":TRIG:EDGE:LEV " + compose("%.9g",triggerLevel)
    ":TRIG:SWE NORM"
    ];
for command = commands.'
    writeline(scope,command);
end
end

function captureScreen(scope,fileName)
writeline(scope,":DISP:DATA? ON,OFF,PNG");
bytes = readbinblock(scope,"uint8");
fid = fopen(fileName,"w");
assert(fid~=-1,"No se pudo crear la captura PNG.");
cleanupFile = onCleanup(@() fclose(fid));
fwrite(fid,bytes,"uint8");
end

function T = captureWaveform(scope,channel)
writeline(scope,":WAV:SOUR " + channel);
writeline(scope,":WAV:MODE NORM");
writeline(scope,":WAV:FORM BYTE");
preamble = str2double(split(strip(writeread(scope,":WAV:PRE?")),","));
writeline(scope,":WAV:DATA?");
raw = double(readbinblock(scope,"uint8"));
assert(numel(preamble)>=10,"Preambulo de onda RIGOL incompleto.");
sample = (0:numel(raw)-1).';
time_s = (sample-preamble(7)).*preamble(5)+preamble(6);
voltage_V = (raw(:)-preamble(9)-preamble(10)).*preamble(8);
T = table(sample,time_s,raw(:),voltage_V, ...
    VariableNames=["sample","time_s","raw","voltage_V"]);
end

function T = parseUartTable(lines)
headerIndex = find(startsWith(lines,"k,t_us,"),1,"first");
endIndex = find(startsWith(lines,"# END"),1,"first");
assert(~isempty(headerIndex) && ~isempty(endIndex) && endIndex>headerIndex, ...
    "No se encontro la tabla UART completa.");
body = lines(headerIndex:endIndex-1);
tmp = string(tempname) + ".csv";
cleanupTmp = onCleanup(@() deleteIfExists(tmp));
writeTextLines(tmp,body);
T = readtable(tmp,VariableNamingRule="preserve");
end

function writeTextLines(fileName,lines)
fid = fopen(fileName,"w");
assert(fid~=-1,"No se pudo crear %s.",fileName);
cleanupFile = onCleanup(@() fclose(fid));
for line = lines.'
    fprintf(fid,"%s\n",line);
end
end

function deleteIfExists(fileName)
if isfile(fileName)
    delete(fileName);
end
end
