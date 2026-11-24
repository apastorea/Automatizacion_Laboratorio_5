# Laboratorio 5 — Síntesis directa e intermuestreo

Entrega mínima y reproducible del Laboratorio 5 de Automatización.

## Contenido

- `Informe/Informe_Lab5_Automatizacion_Pastore.pdf`: informe final en formato IEEE.
- `Informe/LaTeX/`: fuente del informe y figuras utilizadas.
- `PSoC/`: proyecto `Lab5` para PSoC Creator, con los cuatro controladores implementados.
- `MATLAB/`: diseño, simulación, adquisición y revisión experimental.

## Reproducción

1. Ejecutar `MATLAB/lab5_puntos_1_2.m` para recalcular los controladores y las simulaciones.
2. Abrir `PSoC/Lab5.cywrk` en PSoC Creator. Seleccionar el modo y el controlador en `lab5_experiment.h`, generar la aplicación y programar la placa.
3. Para medir, conectar el canal 1 del osciloscopio a J5 (esfuerzo de control) y el canal 2 a J7 (salida de la planta). El escalón de ensayo es de 50 mV.
4. Ejecutar `MATLAB/lab5_captura_monitor_rigol.m` para adquirir una prueba y `MATLAB/lab5_analizar_control_experimental.m` para procesarla.
5. Ejecutar `MATLAB/lab5_revision_correccion_docente.m` para reconstruir las métricas y comparaciones continuas de los cuatro ensayos válidos incluidos.

El proyecto PSoC se entrega en modo `MONITOR` como estado seguro. Los directorios generados por compilación se recrean desde PSoC Creator y no forman parte del repositorio.

## Ensayos incluidos

- TS1 oscilatorio y TS1 deadbeat, repetidos con disparo físico del osciloscopio.
- TS2 oscilatorio y TS2 deadbeat.

Cada carpeta experimental contiene cabecera UART, 256 muestras del PSoC, trazas continuas J5/J7 y metadatos del ensayo. No se incluyen ensayos preliminares ni capturas descartadas.
