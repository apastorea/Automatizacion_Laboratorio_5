function result = lab5_analizar_control_experimental(dataFolder,samplingCase,design)
%LAB5_ANALIZAR_CONTROL_EXPERIMENTAL Compara control real y modelo identificado.
%   Conserva las magnitudes fisicas de la UART y la correccion de cero que
%   el firmware estima con las muestras de pretrigger. No normaliza las
%   senales. La simulacion usa el mismo escalon que el experimento.

arguments
    dataFolder (1,1) string {mustBeFolder}
    samplingCase (1,1) double {mustBeInteger,mustBePositive} = 1
    design (1,1) string {mustBeMember(design,["oscilatorio","deadbeat"])} = "deadbeat"
end

scriptFolder = fileparts(mfilename("fullpath"));
modelFile = fullfile(scriptFolder,"resultados_puntos_1_2","lab5_puntos_1_2.mat");
uart = readtable(fullfile(dataFolder,"psoc_uart.csv"));
modelData = load(modelFile,"results");

requiredVariables = ["k","t_us","r_mV","y_mV","u_requested_mV", ...
    "u_applied_mV","flags"];
assert(all(ismember(requiredVariables,string(uart.Properties.VariableNames))), ...
    "La tabla UART no contiene todas las variables requeridas.");
assert(height(uart)>=64,"La captura de control es demasiado corta.");
assert(all(uart.flags==0),"La captura contiene banderas de fallo.");

pretrigger = find(uart.r_mV~=0,1,"first") - 1;
assert(~isempty(pretrigger) && pretrigger>=8, ...
    "No se pudo identificar el escalon de referencia.");
preRows = uart.k<pretrigger;
postRows = uart.k>=pretrigger;

TsExp = median(diff(uart.t_us))*1e-6;
referenceV = median(uart.r_mV(postRows))/1000;
assert(max(abs(uart.r_mV(postRows)/1000-referenceV))<1e-9, ...
    "La referencia experimental no es un escalon de amplitud constante.");

runtimeZeroOffsetV = mean(uart.y_mV(preRows),"omitmissing")/1000;
yInitialV = 0;
effectiveStepV = referenceV;
yExpV = uart.y_mV(postRows)/1000;
uRequestedV = uart.u_requested_mV(postRows)/1000;
uAppliedV = uart.u_applied_mV(postRows)/1000;
tExpS = (uart.t_us(postRows)-uart.t_us(find(postRows,1,"first")))*1e-6;

results = modelData.results;
caseMask = [results.sampling_case]==samplingCase & ...
    string({results.design})==design;
assert(nnz(caseMask)==1,"No se encontro exactamente un modelo para el caso solicitado.");
model = results(caseMask);
assert(abs(model.Ts_s-TsExp)<=2e-6, ...
    "El periodo experimental no coincide con el modelo seleccionado.");

n = nnz(postRows);
tModelS = (0:n-1).'*model.Ts_s;
rDeviationV = effectiveStepV*ones(n,1);
yModelV = yInitialV + lsim(model.T_closed_loop,rDeviationV,tModelS);
uModelV = lsim(model.U_over_R,rDeviationV,tModelS);

steadyCount = min(64,n);
steadyRows = (n-steadyCount+1:n).';
ySteadyMeanV = mean(yExpV(steadyRows),"omitmissing");
ySteadyP2pV = max(yExpV(steadyRows))-min(yExpV(steadyRows));
settling10S = settlingTime(tExpS,yExpV,referenceV,0.10*abs(effectiveStepV));

metric = ["sample_period_experimental";"reference_experimental"; ...
    "reference_simulation";"runtime_zero_offset"; ...
    "initial_output_experimental"; ...
    "effective_step_simulation";"output_peak_experimental"; ...
    "control_peak_applied_experimental";"output_steady_mean_experimental"; ...
    "output_steady_p2p_experimental";"output_rmse_model_vs_experiment"; ...
    "control_rmse_model_vs_applied";"control_rmse_model_vs_requested"; ...
    "settling_time_10pct_experimental";"flagged_samples"];
value = [TsExp;referenceV;referenceV;runtimeZeroOffsetV; ...
    yInitialV;effectiveStepV; ...
    max(yExpV);max(abs(uAppliedV));ySteadyMeanV;ySteadyP2pV; ...
    sqrt(mean((yModelV-yExpV).^2)); ...
    sqrt(mean((uModelV-uAppliedV).^2)); ...
    sqrt(mean((uModelV-uRequestedV).^2));settling10S;nnz(uart.flags)];
unit = ["s";"V";"V";"V";"V";"V";"V";"V";"V";"V"; ...
    "V";"V";"V";"s";"count"];
metrics = table(metric,value,unit);

aligned = table(tExpS,referenceV*ones(n,1),yExpV,yModelV, ...
    uRequestedV,uAppliedV,uModelV, ...
    VariableNames=["time_s","reference_V","y_experimental_V", ...
    "y_model_V","u_requested_V","u_applied_V","u_model_V"]);
aligned.Properties.VariableUnits = ["s","V","V","V","V","V","V"];

writetable(metrics,fullfile(dataFolder,"metricas_matlab_experimento.csv"));
writetable(aligned,fullfile(dataFolder,"trazas_alineadas_matlab_experimento.csv"));

fig = figure(Color="w",Position=[80 80 1350 850]);
layout = tiledlayout(fig,2,1,TileSpacing="compact",Padding="compact");

axOutput = nexttile(layout);
plot(axOutput,1e3*tModelS,1e3*yModelV,LineWidth=2.0, ...
    DisplayName="MATLAB identificado con y(0) experimental");
hold(axOutput,"on");
stairs(axOutput,1e3*tExpS,1e3*yExpV,LineWidth=1.25, ...
    DisplayName="PSoC/ADC experimental");
yline(axOutput,1e3*referenceV,"--",LineWidth=1.2, ...
    Label=sprintf("Referencia = %.0f mV",1e3*referenceV), ...
    DisplayName="Referencia comun");
grid(axOutput,"on");
ylabel(axOutput,"Salida y (mV)");
title(axOutput,sprintf("Salida: T_s = %.3f ms, %s",1e3*TsExp,design));
legend(axOutput,Location="best",NumColumns=2);
text(axOutput,0.99,0.05, ...
    sprintf("Cero de pretrigger = %.2f mV; sin normalizacion\nRMSE = %.2f mV", ...
    1e3*runtimeZeroOffsetV,1e3*sqrt(mean((yModelV-yExpV).^2))), ...
    Units="normalized",HorizontalAlignment="right", ...
    VerticalAlignment="bottom",BackgroundColor="w",Margin=4);

axControl = nexttile(layout);
plot(axControl,1e3*tModelS,1e3*uModelV,LineWidth=2.0, ...
    DisplayName="MATLAB");
hold(axControl,"on");
stairs(axControl,1e3*tExpS,1e3*uRequestedV,":",LineWidth=1.2, ...
    DisplayName="PSoC solicitado");
stairs(axControl,1e3*tExpS,1e3*uAppliedV,LineWidth=1.25, ...
    DisplayName="PSoC aplicado (cuantizado)");
grid(axControl,"on");
xlabel(axControl,"Tiempo desde el escalon (ms)");
ylabel(axControl,"Esfuerzo u (mV)");
title(axControl,sprintf( ...
    "Esfuerzo de control con la misma referencia de %.0f mV", ...
    1e3*referenceV));
legend(axControl,Location="best",NumColumns=3);
linkaxes([axOutput axControl],"x");
xlim(axOutput,[0 1e3*tExpS(end)]);
title(layout,"Laboratorio 5: comparacion MATLAB--experimento (magnitudes fisicas)", ...
    FontWeight="bold");

fullFigurePath = fullfile(dataFolder,"comparacion_matlab_experimento_completa.png");
exportgraphics(fig,fullFigurePath,Resolution=300);
if samplingCase == 1
    focusWindowMs = 350;
else
    focusWindowMs = 120;
end
xlim(axOutput,[0 min(focusWindowMs,1e3*tExpS(end))]);
figurePath = fullfile(dataFolder,"comparacion_matlab_experimento.png");
exportgraphics(fig,figurePath,Resolution=300);
close(fig);

result = struct("metrics",metrics,"aligned",aligned,"figurePath",figurePath, ...
    "fullFigurePath",fullFigurePath, ...
    "dataFolder",dataFolder,"samplingCase",samplingCase,"design",design, ...
    "runtimeZeroOffsetCorrection",true, ...
    "additionalOffsetCorrection",false,"normalized",false);
disp(metrics)
fprintf("CONTROL_ANALIZADO: %s\n",dataFolder);
end

function value = settlingTime(timeS,signal,reference,tolerance)
outside = find(abs(signal-reference)>tolerance,1,"last");
if isempty(outside)
    value = 0;
elseif outside==numel(signal)
    value = NaN;
else
    value = timeS(outside+1);
end
end
