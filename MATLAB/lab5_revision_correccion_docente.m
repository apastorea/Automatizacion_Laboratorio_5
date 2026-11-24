%% Laboratorio 5 - revisión solicitada por la cátedra
% Audita la identidad de los controladores registrados en cada captura,
% compara las trazas continuas del osciloscopio con el modelo identificado
% y cuantifica la sensibilidad del esfuerzo de control al ruido de medida.

clearvars;
close all;
clc;

scriptFolder = fileparts(mfilename("fullpath"));
dataRoot = fullfile(scriptFolder,"datos_experimentales");
modelFile = fullfile(scriptFolder,"resultados_puntos_1_2", ...
    "lab5_puntos_1_2.mat");
outputFolder = fullfile(scriptFolder,"resultados_revision_docente");
if ~isfolder(outputFolder)
    mkdir(outputFolder);
end

folders = [
    "lab5_TS1_oscillatory_50mV_scopevalido_20261124_184303"
    "lab5_TS1_deadbeat_50mV_scopevalido_20261124_184411"
    "lab5_identificado_TS2_oscillatory_50mV_zerocorr_scopefinal_20261121_180202"
    "lab5_identificado_TS2_deadbeat_50mV_zerocorr_final_20261121_180423"
];
labels = ["TS1 oscilatorio";"TS1 deadbeat"; ...
    "TS2 oscilatorio";"TS2 deadbeat"];
expectedIdentifiers = ["TS1_OSCILLATORY";"TS1_DEADBEAT"; ...
    "TS2_OSCILLATORY";"TS2_DEADBEAT"];
samplingCases = [1;1;2;2];
designs = ["oscilatorio";"deadbeat";"oscilatorio";"deadbeat"];

loaded = load(modelFile,"results");
results = loaded.results;

nCases = numel(folders);
actualIdentifiers = strings(nCases,1);
coefficientLines = strings(nCases,1);
validCapture = false(nCases,1);
scopeControlMinV = nan(nCases,1);
scopeControlMaxV = nan(nCases,1);
scopeOutputMinV = nan(nCases,1);
scopeOutputMaxV = nan(nCases,1);
stepTimeScopeS = nan(nCases,1);

for idx = 1:nCases
    folder = fullfile(dataRoot,folders(idx));
    uartText = fileread(fullfile(folder,"psoc_uart.txt"));
    actualIdentifiers(idx) = extractHeaderValue(uartText,"design");
    coefficientLines(idx) = extractCoefficientLine(uartText);
    validCapture(idx) = actualIdentifiers(idx) == expectedIdentifiers(idx);

    modelMask = [results.sampling_case].' == samplingCases(idx) & ...
        string({results.design}).' == designs(idx);
    assert(nnz(modelMask)==1,"No se encontró el modelo solicitado.");
    model = results(modelMask);

    ch1 = readtable(fullfile(folder,"chan1_J5.csv"));
    ch2 = readtable(fullfile(folder,"chan2_J7.csv"));
    [stepTimeScopeS(idx),controlBaselineV] = detectPhysicalStep( ...
        ch1.time_s,ch1.voltage_V);
    preRows = ch2.time_s < stepTimeScopeS(idx)-1e-3;
    if nnz(preRows)<5
        preRows = (1:min(10,height(ch2))).';
    end
    outputBaselineV = median(ch2.voltage_V(preRows),"omitmissing");

    postRows1 = ch1.time_s >= stepTimeScopeS(idx);
    postRows2 = ch2.time_s >= stepTimeScopeS(idx);
    scopeControlMinV(idx) = min(ch1.voltage_V(postRows1),[],"omitmissing");
    scopeControlMaxV(idx) = max(ch1.voltage_V(postRows1),[],"omitmissing");
    scopeOutputMinV(idx) = min(ch2.voltage_V(postRows2),[],"omitmissing");
    scopeOutputMaxV(idx) = max(ch2.voltage_V(postRows2),[],"omitmissing");

    makeContinuousComparison(model,ch1,ch2,stepTimeScopeS(idx), ...
        controlBaselineV,outputBaselineV,labels(idx),validCapture(idx), ...
        actualIdentifiers(idx),outputFolder);
end

audit = table(labels,expectedIdentifiers,actualIdentifiers,validCapture, ...
    coefficientLines,stepTimeScopeS,scopeControlMinV,scopeControlMaxV, ...
    scopeOutputMinV,scopeOutputMaxV, ...
    VariableNames=["caso","identificador_esperado","identificador_registrado", ...
    "captura_valida","coeficientes_registrados","instante_escalon_scope_s", ...
    "u_scope_min_V","u_scope_max_V","y_scope_min_V","y_scope_max_V"]);
writetable(audit,fullfile(outputFolder,"auditoria_capturas.csv"));

% Transferencia desde ruido de medición hasta esfuerzo de control:
% U(z)/N(z) = -C(z)/(1+C(z)Gd(z)). Se informa la magnitud máxima y la
% magnitud en Nyquist; ambas permiten comparar la amplificación de ruido.
noiseCase = strings(numel(results),1);
plantZero = nan(numel(results),1);
noisePeakGain = nan(numel(results),1);
noiseNyquistGain = nan(numel(results),1);
for idx = 1:numel(results)
    model = results(idx);
    noiseCase(idx) = sprintf("TS%d %s",model.sampling_case,model.design);
    plantZero(idx) = model.plant_zero;
    noiseToControl = minreal(-feedback(model.C,model.Gd),1e-9);
    omega = linspace(0,pi/model.Ts_s,4000);
    response = squeeze(freqresp(noiseToControl,omega));
    noisePeakGain(idx) = max(abs(response));
    noiseNyquistGain(idx) = abs(response(end));
end
noiseSensitivity = table(noiseCase,plantZero,noisePeakGain, ...
    noiseNyquistGain,VariableNames=["caso","cero_planta", ...
    "ganancia_max_ruido_a_control","ganancia_nyquist_ruido_a_control"]);
writetable(noiseSensitivity,fullfile(outputFolder, ...
    "sensibilidad_ruido_control.csv"));

oscTs2 = labels=="TS2 oscilatorio";
deadbeatTs2 = labels=="TS2 deadbeat";
reduction = 100*(scopeControlMaxV(oscTs2)-scopeControlMaxV(deadbeatTs2)) ...
    /scopeControlMaxV(oscTs2);
summaryMetrics = table(scopeControlMaxV(oscTs2), ...
    scopeControlMaxV(deadbeatTs2),reduction, ...
    VariableNames=["u_max_TS2_oscilatorio_V","u_max_TS2_deadbeat_V", ...
    "reduccion_porcentual"]);
writetable(summaryMetrics,fullfile(outputFolder,"metricas_corregidas.csv"));

disp(audit(:,["caso","identificador_esperado", ...
    "identificador_registrado","captura_valida"]));
disp(noiseSensitivity);
disp(summaryMetrics);
fprintf("REVISIÓN_GENERADA: %s\n",outputFolder);

function value = extractHeaderValue(text,field)
expression = field + "=([A-Z0-9_]+)";
token = regexp(text,expression,"tokens","once");
assert(~isempty(token),"No se encontró %s en la cabecera UART.",field);
value = string(token{1});
end

function value = extractCoefficientLine(text)
line = regexp(text,"# b0_x1e6=.*a2_x1e6=[-0-9]+", ...
    "match","once");
assert(~isempty(line),"No se encontró la línea de coeficientes.");
value = string(erase(line,"# "));
end

function [stepTime,baseline] = detectPhysicalStep(time,control)
nBaseline = min(10,numel(control));
baseline = median(control(1:nBaseline),"omitmissing");
threshold = max(0.03,6*std(control(1:nBaseline),0,"omitmissing"));
stepIndex = find(abs(control-baseline)>threshold,1,"first");
assert(~isempty(stepIndex),"No se detectó el escalón físico en CH1.");
stepTime = time(stepIndex);
preRows = time < stepTime-1e-3;
if nnz(preRows)>=5
    baseline = median(control(preRows),"omitmissing");
end
end

function makeContinuousComparison(model,ch1,ch2,stepTime,controlBaseline, ...
    outputBaseline,label,isValid,actualIdentifier,outputFolder)
if ~isValid
    controlBaseline = 0;
    outputBaseline = 0;
end
tScopeMs = 1e3*(ch1.time_s-stepTime);
tScopeOutputMs = 1e3*(ch2.time_s-stepTime);
uContinuous = interp1(model.t_samples,model.u_samples_v,model.t_cont, ...
    "previous","extrap");
tModelMs = 1e3*model.t_cont;
uModelPhysicalMv = 1e3*(controlBaseline+uContinuous);
yModelPhysicalMv = 1e3*(outputBaseline+model.y_cont_v);
referencePhysicalMv = 1e3*(outputBaseline+model.reference_step_v);

if model.sampling_case==1
    focusWindowMs = 35;
else
    focusWindowMs = 20;
end

figureHandle = figure(Color="w",Position=[100 100 1280 780]);
layout = tiledlayout(figureHandle,2,1,TileSpacing="compact", ...
    Padding="compact");

outputAxes = nexttile(layout);
hOutputScope = plot(outputAxes,tScopeOutputMs,1e3*ch2.voltage_V,Color=[0.00 0.45 0.74], ...
    LineWidth=1.0,DisplayName="Osciloscopio CH2/J7");
hold(outputAxes,"on");
hOutputModel = plot(outputAxes,tModelMs,yModelPhysicalMv,"--",Color=[0.85 0.33 0.10], ...
    LineWidth=1.8,DisplayName="Modelo continuo");
hReference = yline(outputAxes,referencePhysicalMv,":",Color=[0.20 0.20 0.20], ...
    LineWidth=1.1,DisplayName="Referencia física");
grid(outputAxes,"on");
ylabel(outputAxes,"Salida y (mV)");
outputAxes.FontName = "Times New Roman";
outputAxes.FontSize = 10;

controlAxes = nexttile(layout);
hControlScope = plot(controlAxes,tScopeMs,1e3*ch1.voltage_V,Color=[0.49 0.18 0.56], ...
    LineWidth=1.0,DisplayName="Osciloscopio CH1/J5");
hold(controlAxes,"on");
hControlModel = plot(controlAxes,tModelMs,uModelPhysicalMv,"--",Color=[0.47 0.67 0.19], ...
    LineWidth=1.8,DisplayName="Modelo con ZOH");
grid(controlAxes,"on");
xlabel(controlAxes,"Tiempo desde el escalón (ms)");
ylabel(controlAxes,"Esfuerzo u (mV)");
controlAxes.FontName = "Times New Roman";
controlAxes.FontSize = 10;
linkaxes([outputAxes controlAxes],"x");
xlim(outputAxes,[-2 focusWindowMs]);

if isValid
    legend(outputAxes,[hOutputScope hOutputModel hReference], ...
        Location="best",NumColumns=3);
    legend(controlAxes,[hControlScope hControlModel], ...
        Location="best",NumColumns=2);
    title(layout,"Comparación continua simulación–osciloscopio: " + label, ...
        FontName="Times New Roman",FontWeight="bold");
else
    hOutputScope.Visible = "off";
    hControlScope.Visible = "off";
    legend(outputAxes,[hOutputModel hReference],Location="best",NumColumns=2);
    legend(controlAxes,hControlModel,Location="best");
    title(layout,"TS1 deadbeat sin validación experimental", ...
        FontName="Times New Roman",FontWeight="bold");
    subtitle(layout,"El archivo adjunto registra " + actualIdentifier + ...
        "; las curvas experimentales no se atribuyen al diseño deadbeat", ...
        FontName="Times New Roman");
end

fileTag = replace(lower(label)," ","_");
fileTag = replace(fileTag,"ó","o");
exportgraphics(figureHandle,fullfile(outputFolder, ...
    "comparacion_continua_"+fileTag+".png"),Resolution=300);
close(figureHandle);
end
