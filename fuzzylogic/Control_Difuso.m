function gui_distancia_plot()
    % Variables internas
    s = [];
    tSerial = [];
    conectado = false;
    buffer = "";
    tiempo = [];
    distancias = [];
    errores = [];
    angulos = [];
    contador = 0;
    setpoint = 25;  % en cm por defecto
    error_ant = 0;
    fis = crearFIS();

    % Crear GUI
    f = uifigure('Name','Lectura de Distancia','Position',[100 100 950 850]);

    % Cierre seguro
    f.CloseRequestFcn = @(src, event) cerrarGUI();

    % Selector de puerto COM
    uilabel(f, 'Text', 'Puerto COM:', 'Position', [30 800 80 22]);
    puertos = serialportlist("all");
    popup_puertos = uidropdown(f, 'Items', puertos, 'Position', [110 800 100 22]);

    % Botón conectar
    btn_conectar = uibutton(f, 'Text', 'Conectar', 'Position', [230 800 100 22], ...
        'ButtonPushedFcn', @(~,~) conectar());

    % Indicador de estado
    uilabel(f, 'Text', 'Estado:', 'Position', [350 800 50 22]);
    foco = uilamp(f, 'Color', [1 0 0], 'Position', [410 800 20 20]);

    % Slider de setpoint
    uilabel(f, 'Text', 'Setpoint (cm):', 'Position', [30 760 90 22]);
    slider = uislider(f, 'Limits', [10 35], 'Value', 25, 'Position', [130 770 200 3]);
    btn_setpoint = uibutton(f, 'Text', 'Establecer', 'Position', [350 760 100 22], ...
        'ButtonPushedFcn', @(~,~) establecerSetpoint());

    % Ejes de gráficas en tiempo real
    ax1 = uiaxes(f, 'Position', [30 520 280 200]);
    title(ax1,'Distancia vs Tiempo'); xlabel(ax1,'Tiempo (s)'); ylabel(ax1,'Distancia (cm)');

    ax2 = uiaxes(f, 'Position', [340 520 280 200]);
    title(ax2,'Ángulo vs Tiempo'); xlabel(ax2,'Tiempo (s)'); ylabel(ax2,'Ángulo (°)');

    ax3 = uiaxes(f, 'Position', [650 520 250 200]);
    title(ax3,'Error vs Tiempo'); xlabel(ax3,'Tiempo (s)'); ylabel(ax3,'Error (cm)');

    function establecerSetpoint()
        setpoint = slider.Value;
        errores = [];
        tiempo = [];
        distancias = [];
        angulos = [];
        contador = 0;
        error_ant = 0;
        cla(ax1); cla(ax2); cla(ax3);
        disp("Nuevo setpoint: " + setpoint + " cm");
    end

    function conectar()
        puerto = popup_puertos.Value;
        try
            s = serialport(puerto, 115200);
            configureTerminator(s, "LF");
            pause(2);
            flush(s);
            conectado = true;
            foco.Color = [0 1 0];
            disp("Conectado a " + puerto);
            iniciarTimer();
        catch ME
            conectado = false;
            foco.Color = [1 0 0];
            uialert(f, "Error al conectar al puerto.", "Error");
            disp("Error: " + ME.message);
        end
    end

    function iniciarTimer()
        tSerial = timer;
        tSerial.ExecutionMode = 'fixedSpacing';
        tSerial.Period = 0.1;
        tSerial.TimerFcn = @(~,~) leerSerial();
        tSerial.ErrorFcn = @(~,e) disp("Error en timer: " + e.Data.Message);
        start(tSerial);
    end
    

    distancia_anterior = 0;

    function leerSerial()
        try
            if conectado && s.NumBytesAvailable > 0
                datos = read(s, s.NumBytesAvailable, "uint8");
                texto = char(datos);
                buffer = buffer + string(texto);
    
                idx = strfind(buffer, newline);
                while ~isempty(idx)
                    linea = strtrim(extractBefore(buffer, idx(1)));
                    buffer = extractAfter(buffer, idx(1));
    
                    partes = split(linea, ",");
                    if numel(partes) == 2
                        dist_mm = str2double(partes{1});
                        angulo = str2double(partes{2});
    
                        if ~isnan(dist_mm) && ~isnan(angulo)
                            distancia = dist_mm / 10;
    
                            % Filtro exponencial
                            if contador == 0
                                distancia_anterior = distancia;
                            end
                            distancia_filtrada = 0.47 * distancia + 0.53 * distancia_anterior;
                            distancia_anterior = distancia_filtrada;
    
                            % Control difuso (distancia filtrada)
                            error_actual = setpoint - distancia_filtrada;
                            delta_error = error_actual - error_ant;
                            error_ant = error_actual;
    
                            % Recortar valores extremos
                            error_actual = max(-20, min(20, error_actual));
                            delta_error = max(-15, min(15, delta_error));
    
                            salida = evalfis(fis, [error_actual, delta_error]);
                            salida = max(20, min(160, round(salida)));  % Limitar entre 20° y 160°
                            
                            % Compensación tipo map(u, 30, 150, 14, 3)
                            compensacion = interp1([30, 150], [18, 6], salida, 'linear', 'extrap');
                            salida = salida - round(compensacion);
                            salida = max(20, min(160, salida));  % Volver a limitar por seguridad

                            writeline(s, num2str(salida));
    
                            t_actual = datetime('now');
                            t_val = seconds(t_actual - tiempo_inicio());
    
                            tiempo(end+1) = t_val;
                            distancias(end+1) = distancia_filtrada;
                            errores(end+1) = error_actual;
                            angulos(end+1) = angulo;
    
                            fprintf("Dist: %.1f cm | Error: %.1f | dError: %.1f | Servo Out: %d\n", ...
                                distancia_filtrada, error_actual, delta_error, salida);
    
                            if mod(contador, 5) == 0
                                plot(ax1, tiempo, distancias, 'b'); grid(ax1, 'on');
                                plot(ax2, tiempo, angulos, 'g'); grid(ax2, 'on');
                                plot(ax3, tiempo, errores, 'r'); grid(ax3, 'on');
                                drawnow limitrate;
                            end
    
                            contador = contador + 1;
                        end
                    end
                    idx = strfind(buffer, newline);
                end
            end
        catch ME
            disp("Error leyendo: " + ME.message);
        end
    end


    function cerrarGUI()
        try
            if ~isempty(tSerial) && isvalid(tSerial)
                stop(tSerial);
                delete(tSerial);
            end
        end
        try
            if ~isempty(s) && isa(s, "serialport")
                clear s;
            end
        end
        delete(f);
    end

    function t0 = tiempo_inicio()
        persistent tstart
        if isempty(tstart)
            tstart = datetime('now');
        end
        t0 = tstart;
    end

    function fis = crearFIS()
        fis = mamfis("Name", "BallAndBeamFIS");

        etiquetas = ["NG", "N", "Z", "P", "PG"];
        x_err = [-20 -10 0 10 20];
        x_dif = [-15 -7.5 0 7.5 15];
        x_out = [170 130 90 50 10]; 

        fis = addInput(fis, [-20 20], "Name", "error");
        for i = 1:5
            fis = addMF(fis, "error", "trimf", triSpan(x_err, i), "Name", etiquetas(i));
        end

        fis = addInput(fis, [-15 15], "Name", "delta_error");
        for i = 1:5
            fis = addMF(fis, "delta_error", "trimf", triSpan(x_dif, i), "Name", etiquetas(i));
        end

        fis = addOutput(fis, [20 160], "Name", "servo");
        for i = 1:5
            fis = addMF(fis, "servo", "trimf", sort(triSpan(x_out, i)), "Name", etiquetas(i));
        end

        reglas_texto = [
            "1 1 1 1 1"; "1 2 1 1 1"; "1 3 1 1 1"; "1 4 2 1 1"; "1 5 3 1 1";
            "2 1 1 1 1"; "2 2 1 1 1"; "2 3 2 1 1"; "2 4 3 1 1"; "2 5 3 1 1";
            "3 1 2 1 1"; "3 2 2 1 1"; "3 3 3 1 1"; "3 4 3 1 1"; "3 5 2 1 1";
            "4 1 3 1 1"; "4 2 3 1 1"; "4 3 4 1 1"; "4 4 4 1 1"; "4 5 5 1 1";
            "5 1 4 1 1"; "5 2 4 1 1"; "5 3 5 1 1"; "5 4 5 1 1"; "5 5 5 1 1"
        ];
        reglas = str2num(char(reglas_texto)); %#ok<ST2NM>
        fis = addRule(fis, reglas);
    end

    function v = triSpan(x, i)
        if i == 1
            v = [x(1) x(1) x(2)];
        elseif i == length(x)
            v = [x(end-1) x(end) x(end)];
        else
            v = [x(i-1) x(i) x(i+1)];
        end
    end
end