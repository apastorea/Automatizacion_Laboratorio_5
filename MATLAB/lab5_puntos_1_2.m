%% Laboratorio 5 - Puntos 1 y 2: modelado y sintesis directa
% Automatizacion - UCA
%
% El script:
%   1) adopta el mejor modelo continuo identificado de la planta;
%   2) selecciona dos periodos de muestreo a partir del ancho de banda;
%   3) discretiza mediante ZOH;
%   4) disena, para cada Ts, un controlador de tiempo minimo con rizado
%      intermuestreo y otro deadbeat sin rizado determinista;
%   5) reconstruye la trayectoria continua entre muestras;
%   6) exporta coeficientes, indicadores, metadatos y figuras legibles.
%
% IMPORTANTE PARA LA COMPARACION EXPERIMENTAL:
% reference_step_v es la amplitud FISICA que debe programarse tambien en el
% PSoC. No comparar un escalon unitario de MATLAB con otro nivel fisico.

clear;
close all;
clc;

%% Configuracion trazable del ensayo
reference_step_v = 0.050;      % Escalon fisico comun para los cuatro casos [V]
output_limit_v = 1.000;        % Limite conservador heredado de Lab. 4 [V]
output_headroom = 0.90;        % Solo se utiliza 90 % del limite
timer_clock_hz = 100e3;        % Clock_1 del PSoC [Hz]
bandwidth_multipliers = [10 30];

% Ganancias heredadas y revalidadas con la captura bipolar final de Lab. 5.
% El offset del actuador se conserva explicitamente y no se oculta en las
% comparaciones entre simulacion y experimento.
calibration.status = "REVALIDADA_LAB5_20261120_231219";
calibration.validation_capture = "lab5_calibracion_final_circuito_cerrado_v4_20261120_231219";
calibration.adc_offset_v = 2.353802067;
calibration.adc_v_per_plant_v = 0.210251951;
calibration.dac_v_per_count = 0.004;
calibration.dac_positive_gain = 3.842612000;
calibration.dac_negative_gain = 2.640105691;
calibration.actuator_offset_v = 0.017209;
calibration.actuator_slope = 0.98171;
calibration.actuator_fit_rmse_v = 0.00096675;
calibration.readback_slope = 1.0170;
calibration.readback_intercept_v = 8.1355e-05;
calibration.readback_fit_rmse_v = 0.0033743;

%% Punto 1 - Mejor modelo continuo identificado en los laboratorios previos
R5 = 150e3;
R6 = 47e3;
R1 = 15e3;
R2 = 47e3;
C1 = 100e-9;
C3 = 100e-9;

K_nominal = (R5/R6)*(R1/R2);
tau_slow_nominal = R5*C1;
tau_fast_nominal = R1*C3;

K = 1.002941;
tau_slow = 9.06915e-3;
tau_fast = 0.774399e-3;

s = tf('s');
Gp = K/((tau_slow*s + 1)*(tau_fast*s + 1));
Gp = minreal(Gp);
[num_p,den_p] = tfdata(Gp,'v');
poles_p = pole(Gp);
omega_bandwidth = bandwidth(Gp);
f_bandwidth_hz = omega_bandwidth/(2*pi);

%% Punto 2 - Seleccion de Ts antes de discretizar
Ts_ideal = 1./(bandwidth_multipliers*f_bandwidth_hz);
timer_counts = round(Ts_ideal*timer_clock_hz);
timer_period_register = timer_counts - 1;
Ts = timer_counts/timer_clock_hz;
fs = 1./Ts;

%% Carpeta de resultados
script_dir = fileparts(mfilename('fullpath'));
outdir = fullfile(script_dir,'resultados_puntos_1_2');
if ~exist(outdir,'dir')
    mkdir(outdir);
end

%% Diseno y simulacion de los cuatro casos
case_count = 2*numel(Ts);
results = repmat(empty_result(),case_count,1);
case_index = 0;

for i = 1:numel(Ts)
    Gd = c2d(Gp,Ts(i),'zoh');
    [num_gd,den_gd] = tfdata(Gd,'v');
    num_gd = trim_leading_zeros(num_gd);
    den_gd = den_gd/den_gd(1);

    assert(numel(num_gd) == 2, ...
        'Se esperaba un cero finito y un retardo de una muestra.');
    assert(all(abs(zero(Gd)) < 1), ...
        'El diseno deadbeat propuesto exige ceros estables.');

    z = tf('z',Ts(i));

    % Tiempo minimo en las muestras: y[k] llega en una muestra. No se
    % fuerza que u[k] sea constante en tiempo finito.
    F_osc = 1/z;
    C_osc = minreal(F_osc/(Gd*(1-F_osc)),1e-9);

    % Deadbeat sin rizado: se conserva el numerador estable de Gd dentro de
    % F(z). Asi F/G no contiene el cero de planta en el denominador y u[k]
    % se vuelve constante en tiempo finito para un escalon ideal.
    b1 = num_gd(1);
    b2 = num_gd(2);
    F_deadbeat = tf([b1 b2],[(b1+b2) 0 0],Ts(i));
    C_deadbeat = minreal(F_deadbeat/(Gd*(1-F_deadbeat)),1e-9);

    case_index = case_index + 1;
    results(case_index) = analyze_design(Gp,Gd,C_osc,F_osc,Ts(i), ...
        i,"oscilatorio",reference_step_v,outdir);

    case_index = case_index + 1;
    results(case_index) = analyze_design(Gp,Gd,C_deadbeat,F_deadbeat,Ts(i), ...
        i,"deadbeat",reference_step_v,outdir);

    plot_pz_readable(Gd,sprintf('Planta discreta - T_s = %.3f ms',1e3*Ts(i)), ...
        fullfile(outdir,sprintf('pz_planta_Ts%d.png',i)));
end

%% Verificaciones automaticas del abordaje
for i = 1:numel(results)
    assert(results(i).closed_loop_error_inf < 1e-8, ...
        'La funcion de lazo cerrado no coincide con F(z).');
    assert(abs(results(i).dc_gain_closed_loop - 1) < 1e-8, ...
        'El lazo cerrado no tiene ganancia estatica unitaria.');
end

deadbeat_mask = [results.design] == "deadbeat";
oscillatory_mask = [results.design] == "oscilatorio";
assert(all([results(deadbeat_mask).control_tail_ripple_unit] < 1e-8), ...
    'El esfuerzo deadbeat no se vuelve constante numericamente.');
assert(all([results(oscillatory_mask).control_tail_ripple_unit] > 1e-8), ...
    'No se detecto la cola alternante esperada en el diseno oscilatorio.');

max_unit_control = max([results.u_peak_abs_unit]);
max_safe_reference_v = output_headroom*output_limit_v/max_unit_control;
assert(reference_step_v <= max_safe_reference_v + 1e-12, ...
    ['La referencia seleccionada supera el margen conservador de salida. ' ...
     'Reduzca reference_step_v.']);

%% Tablas reproducibles
sampling_table = table((1:numel(Ts)).',bandwidth_multipliers.',Ts_ideal.', ...
    timer_counts.',timer_period_register.',Ts.',fs.', ...
    'VariableNames',{'caso','multiplo_fB','Ts_ideal_s','timer_counts', ...
    'timer_period_register','Ts_real_s','fs_hz'});
writetable(sampling_table,fullfile(outdir,'muestreo.csv'));

result_table = struct2table(results);
result_table = removevars(result_table,{ ...
    'Gd','C','F_desired','T_closed_loop','U_over_R','t_cont','y_cont_v', ...
    't_samples','y_samples_v','u_samples_v'});
writetable(result_table,fullfile(outdir,'resumen_controladores.csv'));

coefficient_table = build_coefficient_table(results);
writetable(coefficient_table,fullfile(outdir,'coeficientes_controladores.csv'));

metadata_table = table(reference_step_v,output_limit_v,output_headroom, ...
    max_safe_reference_v,string(calibration.status), ...
    calibration.adc_offset_v,calibration.adc_v_per_plant_v, ...
    calibration.dac_v_per_count,calibration.dac_positive_gain, ...
    calibration.dac_negative_gain,calibration.actuator_offset_v, ...
    calibration.actuator_slope,calibration.actuator_fit_rmse_v, ...
    calibration.readback_slope,calibration.readback_intercept_v, ...
    calibration.readback_fit_rmse_v,string(calibration.validation_capture), ...
    'VariableNames',{'reference_step_v','output_limit_v','output_headroom', ...
    'max_safe_reference_v','calibration_status','adc_offset_v', ...
    'adc_v_per_plant_v','dac_v_per_count','dac_positive_gain', ...
    'dac_negative_gain','actuator_offset_v','actuator_slope', ...
    'actuator_fit_rmse_v','readback_slope','readback_intercept_v', ...
    'readback_fit_rmse_v','validation_capture'});
writetable(metadata_table,fullfile(outdir,'metadatos_ensayo.csv'));

write_text_summary(fullfile(outdir,'resumen.txt'),Gp,num_p,den_p,poles_p, ...
    omega_bandwidth,f_bandwidth_hz,sampling_table,results, ...
    reference_step_v,max_safe_reference_v,calibration);

save(fullfile(outdir,'lab5_puntos_1_2.mat'),'Gp','Ts','results', ...
    'sampling_table','coefficient_table','metadata_table','calibration');

fprintf('\nLaboratorio 5 - puntos 1 y 2 completados.\n');
fprintf('Resultados: %s\n',outdir);
fprintf('Escalon fisico adoptado: %.3f V\n',reference_step_v);
fprintf('Maximo escalon recomendado con %.0f %% de margen: %.6f V\n', ...
    100*output_headroom,max_safe_reference_v);

%% Funciones locales
function r = empty_result()
r = struct( ...
    'sampling_case',NaN,'design',"",'Ts_s',NaN,'fs_hz',NaN, ...
    'plant_zero',NaN,'reference_step_v',NaN, ...
    'rise_time_ms',NaN,'settling_time_ms',NaN,'overshoot_percent',NaN, ...
    'u_max_unit',NaN,'u_min_unit',NaN,'u_peak_abs_unit',NaN, ...
    'u_max_v',NaN,'u_min_v',NaN,'steady_state_error_v',NaN, ...
    'control_tail_ripple_unit',NaN,'closed_loop_error_inf',NaN, ...
    'dc_gain_closed_loop',NaN, ...
    'Gd',[],'C',[],'F_desired',[],'T_closed_loop',[],'U_over_R',[], ...
    't_cont',[],'y_cont_v',[],'t_samples',[],'y_samples_v',[], ...
    'u_samples_v',[]);
end

function r = analyze_design(Gp,Gd,C,F_desired,Ts,sampling_case,design, ...
    reference_step_v,outdir)

T_closed_loop = minreal(feedback(C*Gd,1),1e-9);
U_over_R = minreal(feedback(C,Gd),1e-9);

t_final = max(15*Ts,0.12);
t_samples = (0:Ts:t_final).';
r_samples = reference_step_v*ones(size(t_samples));
y_samples_v = lsim(T_closed_loop,r_samples,t_samples);
u_samples_v = lsim(U_over_R,r_samples,t_samples);

fine_step = Ts/250;
t_cont = (0:fine_step:t_samples(end)).';
u_cont_v = interp1(t_samples,u_samples_v,t_cont,'previous','extrap');
y_cont_v = lsim(Gp,u_cont_v,t_cont);

info = stepinfo(y_cont_v,t_cont,reference_step_v, ...
    'RiseTimeLimits',[0.1 0.9],'SettlingTimeThreshold',0.02);

unit_scale = max(abs(reference_step_v),eps);
u_unit = u_samples_v/unit_scale;
% Desde k = 2 el deadbeat ya debe ser constante. En el diseno oscilatorio
% se observa, en cambio, la cola alternante asociada al cero de la planta.
tail_start = min(3,numel(u_unit));
tail_end = min(8,numel(u_unit));
control_tail_ripple_unit = max(u_unit(tail_start:tail_end)) - ...
    min(u_unit(tail_start:tail_end));

closed_loop_error_inf = norm(minreal(T_closed_loop-F_desired,1e-8),inf);

r = empty_result();
r.sampling_case = sampling_case;
r.design = design;
r.Ts_s = Ts;
r.fs_hz = 1/Ts;
r.plant_zero = zero(Gd);
r.reference_step_v = reference_step_v;
r.rise_time_ms = 1e3*info.RiseTime;
r.settling_time_ms = 1e3*info.SettlingTime;
r.overshoot_percent = info.Overshoot;
r.u_max_unit = max(u_unit);
r.u_min_unit = min(u_unit);
r.u_peak_abs_unit = max(abs(u_unit));
r.u_max_v = max(u_samples_v);
r.u_min_v = min(u_samples_v);
r.steady_state_error_v = reference_step_v-y_cont_v(end);
r.control_tail_ripple_unit = control_tail_ripple_unit;
r.closed_loop_error_inf = closed_loop_error_inf;
r.dc_gain_closed_loop = dcgain(T_closed_loop);
r.Gd = Gd;
r.C = C;
r.F_desired = F_desired;
r.T_closed_loop = T_closed_loop;
r.U_over_R = U_over_R;
r.t_cont = t_cont;
r.y_cont_v = y_cont_v;
r.t_samples = t_samples;
r.y_samples_v = y_samples_v;
r.u_samples_v = u_samples_v;

tag = sprintf('Ts%d_%s',sampling_case,char(design));
plot_response(r,fullfile(outdir,['respuesta_' tag '.png']));
plot_pz_readable(C, ...
    sprintf('Controlador %s - T_s = %.3f ms',char(design),1e3*Ts), ...
    fullfile(outdir,['pz_controlador_' tag '.png']));
plot_pz_readable(T_closed_loop, ...
    sprintf('Lazo cerrado %s - T_s = %.3f ms',char(design),1e3*Ts), ...
    fullfile(outdir,['pz_lazo_' tag '.png']));
end

function plot_response(r,filepath)
fig = figure('Color','w','Position',[100 100 1350 850], ...
    'Name',sprintf('Respuesta %s',char(r.design)));
tl = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');

ax1 = nexttile(tl);
plot(ax1,1e3*r.t_cont,r.y_cont_v,'LineWidth',2.0, ...
    'Color',[0.00 0.45 0.74]);
hold(ax1,'on');
stairs(ax1,1e3*r.t_samples,r.reference_step_v*ones(size(r.t_samples)), ...
    '--','LineWidth',1.5,'Color',[0.47 0.67 0.19]);
plot(ax1,1e3*r.t_samples,r.y_samples_v,'o','MarkerSize',5, ...
    'MarkerFaceColor','w','LineWidth',1.2,'Color',[0.85 0.33 0.10]);
hold(ax1,'off');
configure_axes(ax1);
ylabel(ax1,'Salida y [V]');
title(ax1,sprintf('Salida fisica - escalon aplicado = %.3f V', ...
    r.reference_step_v));
legend(ax1,{'Trayectoria continua','Referencia fisica','Muestras y[k]'}, ...
    'Location','southeast','Box','off');

ax2 = nexttile(tl);
stairs(ax2,1e3*r.t_samples,r.u_samples_v,'LineWidth',2.0, ...
    'Color',[0.49 0.18 0.56]);
configure_axes(ax2);
xlabel(ax2,'Tiempo [ms]');
ylabel(ax2,'Esfuerzo u [V]');
title(ax2,'Esfuerzo de control para la misma amplitud fisica');
xlim(ax2,[0 1e3*r.t_samples(end)]);
linkaxes([ax1 ax2],'x');
display_end_ms = min(1e3*r.t_samples(end), ...
    max(1e3*5*r.Ts_s,1.5*r.settling_time_ms));
xlim(ax1,[0 display_end_ms]);
title(tl,sprintf('%s, T_s = %.3f ms',capitalize(r.design),1e3*r.Ts_s), ...
    'FontWeight','bold');
exportgraphics(fig,filepath,'Resolution',300);
close(fig);
end

function plot_pz_readable(sys,plot_title,filepath)
p = pole(sys);
z = zero(sys);
fig = figure('Color','w','Position',[100 100 1050 760], ...
    'Name',plot_title);
ax = axes(fig);
theta = linspace(0,2*pi,800);
handles = gobjects(0);
labels = {};
handles(end+1) = plot(ax,cos(theta),sin(theta),':','Color',[0.45 0.45 0.45], ...
    'LineWidth',1.3);
labels{end+1} = 'Circulo unitario';
hold(ax,'on');
handles(end+1) = xline(ax,0,'Color',[0.75 0.75 0.75]);
labels{end+1} = 'Re = 0';
handles(end+1) = yline(ax,0,'Color',[0.75 0.75 0.75]);
labels{end+1} = 'Im = 0';
if ~isempty(z)
    handles(end+1) = plot(ax,real(z),imag(z),'o','MarkerSize',12,'LineWidth',2.2, ...
        'MarkerFaceColor','w','Color',[0.00 0.45 0.74]);
    labels{end+1} = 'Ceros';
end
if ~isempty(p)
    handles(end+1) = plot(ax,real(p),imag(p),'x','MarkerSize',13,'LineWidth',2.5, ...
        'Color',[0.85 0.33 0.10]);
    labels{end+1} = 'Polos';
end
label_roots(ax,z,'z',0.085);
label_roots(ax,p,'p',-0.095);
hold(ax,'off');
axis(ax,'equal');
xlim(ax,[-1.15 1.15]);
ylim(ax,[-1.05 1.05]);
configure_axes(ax);
xlabel(ax,'Parte real');
ylabel(ax,'Parte imaginaria');
title(ax,plot_title);
legend(ax,handles,labels, ...
    'Location','southoutside','NumColumns',5,'Box','off');
exportgraphics(fig,filepath,'Resolution',300);
close(fig);
end

function label_roots(ax,roots,prefix,y_offset)
if isempty(roots)
    return;
end
roots = roots(:);
unique_roots = roots(1);
for index = 2:numel(roots)
    if all(abs(roots(index)-unique_roots) >= 1e-9)
        unique_roots(end+1,1) = roots(index); %#ok<AGROW>
    end
end
for k = 1:numel(unique_roots)
    multiplicity = sum(abs(roots-unique_roots(k)) < 1e-8);
    if multiplicity > 1
        label = sprintf('%s = %.4f%+.4fj (x%d)',prefix, ...
            real(unique_roots(k)),imag(unique_roots(k)),multiplicity);
    else
        label = sprintf('%s = %.4f%+.4fj',prefix, ...
            real(unique_roots(k)),imag(unique_roots(k)));
    end
    text(ax,real(unique_roots(k))+0.025,imag(unique_roots(k))+y_offset,label, ...
        'FontSize',10,'Interpreter','none','BackgroundColor','w', ...
        'Margin',1,'Clipping','on');
end
end

function configure_axes(ax)
grid(ax,'on');
ax.FontName = 'Arial';
ax.FontSize = 12;
ax.LineWidth = 1.0;
ax.Box = 'on';
ax.TickDir = 'out';
ax.GridAlpha = 0.25;
ax.MinorGridAlpha = 0.12;
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
ax.Layer = 'top';
end

function values = trim_leading_zeros(values)
tolerance = 1e-12*max(1,norm(values,inf));
first = find(abs(values) > tolerance,1,'first');
assert(~isempty(first),'Numerador nulo inesperado.');
values = values(first:end);
end

function t = build_coefficient_table(results)
n = numel(results);
sampling_case = zeros(n,1);
design = strings(n,1);
Ts_s = zeros(n,1);
b0 = zeros(n,1); b1 = zeros(n,1); b2 = zeros(n,1);
a1 = zeros(n,1); a2 = zeros(n,1);
for i = 1:n
    [b,a] = tfdata(results(i).C,'v');
    b = pad_to_three(b);
    a = pad_to_three(a);
    b = b/a(1);
    a = a/a(1);
    sampling_case(i) = results(i).sampling_case;
    design(i) = results(i).design;
    Ts_s(i) = results(i).Ts_s;
    b0(i) = b(1); b1(i) = b(2); b2(i) = b(3);
    a1(i) = a(2); a2(i) = a(3);
end
t = table(sampling_case,design,Ts_s,b0,b1,b2,a1,a2, ...
    'VariableNames',{'sampling_case','design','Ts_s','b0','b1','b2','a1','a2'});
end

function v = pad_to_three(v)
assert(numel(v) <= 3,'Orden de controlador no esperado.');
v = [zeros(1,3-numel(v)) v];
end

function write_text_summary(path,Gp,num_p,den_p,poles_p,omega_b,f_b, ...
    sampling_table,results,reference_step_v,max_safe_reference_v,calibration)
fid = fopen(path,'w');
assert(fid ~= -1,'No se pudo abrir el resumen.');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid,'LABORATORIO 5 - PUNTOS 1 Y 2\n');
fprintf(fid,'================================\n\n');
fprintf(fid,'MODELO CONTINUO NOMINAL\n');
fprintf(fid,'Gp(s) numerador:'); fprintf(fid,' %.12g',num_p); fprintf(fid,'\n');
fprintf(fid,'Gp(s) denominador:'); fprintf(fid,' %.12g',den_p); fprintf(fid,'\n');
fprintf(fid,'Ganancia estatica = %.12g\n',dcgain(Gp));
fprintf(fid,'Polos [rad/s]:'); fprintf(fid,' %.12g',poles_p); fprintf(fid,'\n');
fprintf(fid,'Ancho de banda = %.12g rad/s = %.12g Hz\n\n',omega_b,f_b);

fprintf(fid,'MUESTREO\n');
for i = 1:height(sampling_table)
    fprintf(fid,['Caso %d: criterio %gfB, Ts ideal %.9g s, Ts real %.9g s, ' ...
        'fs %.9g Hz, Period %d\n'],sampling_table.caso(i), ...
        sampling_table.multiplo_fB(i),sampling_table.Ts_ideal_s(i), ...
        sampling_table.Ts_real_s(i),sampling_table.fs_hz(i), ...
        sampling_table.timer_period_register(i));
end

fprintf(fid,'\nAMPLITUD Y CALIBRACION\n');
fprintf(fid,'Escalon fisico usado en TODAS las simulaciones = %.9g V\n', ...
    reference_step_v);
fprintf(fid,'Maximo escalon recomendado con margen = %.9g V\n', ...
    max_safe_reference_v);
fprintf(fid,'Estado de calibracion = %s\n',calibration.status);
fprintf(fid,'ADC: Vadc = offset + escala*y, offset %.12g V, escala %.12g V/V\n', ...
    calibration.adc_offset_v,calibration.adc_v_per_plant_v);
fprintf(fid,['Conversion inversa documentada: y[V] = ' ...
    '(Vadc[V] - offset[V]) / escala[V/V]\n']);
fprintf(fid,['Actuador medido: J5_crudo = %.9g*u_aplicada + %.9g V, ' ...
    'RMSE %.9g V\n'],calibration.actuator_slope, ...
    calibration.actuator_offset_v,calibration.actuator_fit_rmse_v);
fprintf(fid,['Lectura medida: J7_crudo = %.9g*y_ADC + %.9g V, ' ...
    'RMSE %.9g V\n'],calibration.readback_slope, ...
    calibration.readback_intercept_v,calibration.readback_fit_rmse_v);
fprintf(fid,'Captura de validacion = %s\n',calibration.validation_capture);
fprintf(fid,['Las comparaciones deben mostrar el dato crudo y declarar por ' ...
    'separado cualquier correccion de offset.\n\n']);

fprintf(fid,'CONTROLADORES\n');
for i = 1:numel(results)
    [b,a] = tfdata(results(i).C,'v');
    b = pad_to_three(b); a = pad_to_three(a); b = b/a(1); a = a/a(1);
    fprintf(fid,'Caso Ts%d - %s, Ts = %.9g s\n', ...
        results(i).sampling_case,results(i).design,results(i).Ts_s);
    fprintf(fid,'C(z) b ='); fprintf(fid,' %.12g',b); fprintf(fid,'\n');
    fprintf(fid,'C(z) a ='); fprintf(fid,' %.12g',a); fprintf(fid,'\n');
    fprintf(fid,['u[k] = %.12g e[k] + %.12g e[k-1] + %.12g e[k-2] ' ...
        '%+.12g u[k-1] %+.12g u[k-2]\n'], ...
        b(1),b(2),b(3),-a(2),-a(3));
    fprintf(fid,['tr = %.6g ms, ts = %.6g ms, Mp = %.6g %%, ' ...
        'u_unit = [%.6g, %.6g], u_fisico = [%.6g, %.6g] V\n\n'], ...
        results(i).rise_time_ms,results(i).settling_time_ms, ...
        results(i).overshoot_percent,results(i).u_min_unit, ...
        results(i).u_max_unit,results(i).u_min_v,results(i).u_max_v);
end
end

function value = capitalize(value)
value = char(value);
value(1) = upper(value(1));
end
