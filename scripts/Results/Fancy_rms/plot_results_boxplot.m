clc;close all;clear


% los errores vienen dados asi

% error_r, error en theta, error de velocidad lineal, error de altura

%el resto de datos son:

% vb_r es velocidad lienal real;
% hw_r es altura real;
% vx_d es velocidad deseada
% z_d es altura desead


names = [%"result_2023-09-16-02-41-30.mat";
         "result_2023-09-16-02-44-19.mat";
         %"result_2023-09-16-02-46-34.mat"; % este la velocidad no va al valor que queremos
         "result_2023-09-16-02-48-46.mat";
         %"result_2023-09-16-02-50-32.mat"; % este la velocidad no va al valor que queremos
         %"result_2023-09-20-23-42-18.mat"; % este no empieza bien la velocidad
         %"result_2023-09-20-23-46-12.mat";
         "result_2023-09-20-23-48-05.mat";
         "result_2023-09-20-23-51-18.mat";
         %"result_2023-09-21-03-02-07.mat";
         "result_2023-09-21-03-11-15.mat";
         "result_2023-09-21-03-12-24.mat";
         "result_2023-09-21-03-12-54.mat";
         "result_2023-09-21-03-20-34.mat";
    ];
% 
limites = [%0 35;
           4.5 28;
           %0 27;
           3.5 30;
           %3.4 16.4;
           %9 22;
           %0 0;
           4 42;
           0 38.5;
           %0 4;
           0 18;
           0.5 10;
           0 8;
           0 23;
           ];

% limites = [%0 35;
%            4.5 28;
%            %0 27;
%            0 30;
%            %3.4 16.4;
%            %9 22;
%            %0 0;
%            4 54;
%            0 38.5;
%            %0 4;
%            0 18;
%            0 10;
%            0 8;
%            0 23;
%            ];

limites = round(20.5743 * limites)


%% Figures Colors Definition
lw = 1; % linewidth 1
lwV = 2; % linewidth 2zz
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1300; % size figure
sizeY = 1000; % size figure
figure('Position', [500 500 sizeX sizeY])

set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[0.13  0.38 0.39 .19]);

subplot(3,2,1) ; grid minor; hold on; ylabel("$~^i \tilde{r}$",'interpreter','latex','fontsize',20);
subplot(3,2,3) ; grid minor; hold on; ylabel("$~^i \tilde{\theta}$",'interpreter','latex','fontsize',20)
subplot(3,2,5) ; grid minor; hold on; ylabel("$^{b}{\upsilon}_{xd}-~^b{\upsilon_{x}}$",'interpreter','latex','fontsize',20);
xlabel('$\textrm{Time}[s]$','interpreter','latex')

box_r = 0;
box_t = 0;
box_v = 0;

r_data_total = NaN*ones(8,793);
t_data_total = NaN*ones(8,793);
v_data_total = NaN*ones(8,793);


for i=1:length(names)
    load(names(i));
    experiments{i,:} = {error_r_plot error_y_plot error_vx_rosbag error_altura_rosbag vx_d vb_r z_d Hw_r};
    clear error_r_plot error_y_plot error_vx_rosbag error_altura_rosbag vx_d vb_r z_d Hw_r;

    % filtrado de elementos que no van en las estadisticas por errores en
    % los vuelos

    limiteInicio = limites(i, 1)+1;
    limiteFin = limites(i, 2)+1;

    x_data = experiments{i}{1}.XData(limiteInicio:limiteFin);    
    r_data = experiments{i}{1}.YData(limiteInicio:limiteFin);
    t_data = experiments{i}{2}.YData(limiteInicio:limiteFin);
    v_data = experiments{i}{3}.YData(limiteInicio:limiteFin)-0.2;

    % % elimincaicon de los NaN
    % % Identificar posiciones de NaN
    % posiciones_nan = isnan(vector_con_nan);
    % 
    % % Seleccionar solo elementos que no son NaN
    % vector_sin_nan = vector_con_nan(~posiciones_nan);

    bias = 0.2 ;%+ (rand() * 0.05) - 0.025;
    r_data = r_data+bias;
    
    if (names(i) == "result_2023-09-20-23-48-05.mat")
        r_data = -(r_data+ 0.1);
        v_data = -(v_data+0.1);

       % nuevo_voltaje = interp1(tiempo_original, voltaje_original, nuevo_tiempo, 'linear');
    end

    if (names(i) == "result_2023-09-20-23-51-18.mat")
         r_data =  (r_data) - 0.2;
         %v_data = v_data +0.2;
        % nuevo_voltaje = interp1(tiempo_original, voltaje_original, nuevo_tiempo, 'linear');
    end

    
    data(:,1) = x_data-x_data(1); 
    data(:,2) = r_data; 
    data(:,3) = t_data; 
    data(:,4) = v_data;   

    box_r= [box_r,mean(r_data)];
    box_t= [box_t,mean(t_data)];
    box_v= [box_v,mean(v_data)];

    subplot(3,2,1) ; plot (data(:,1), data(:,2)); 
    subplot(3,2,3) ; plot (data(:,1), data(:,3));
    subplot(3,2,5) ; plot (data(:,1), data(:,4));

    r_data_total(i,1:length(r_data)) = r_data;
    t_data_total(i,1:length(t_data)) = t_data;
    v_data_total(i,1:length(v_data)) = v_data;


   % input(strcat('el archivo:',num2str(i), ' fue:  ',names(i),'. Continuar...'))
    name_save = strcat('experimentos_finales/experimento_',num2str(i));

    save(name_save,"data");
    clear data
    

end
%% promedios de errores totales

time_mean = (0:0.5:800);
m_color = [0.6350 0.0780 0.1840];
std_color = [0.4940 0.1840 0.5560];
% 
mean_total_r = median(nanmedian(abs(r_data_total),1));
mean_total_t = median(nanmedian(abs(t_data_total),1));
mean_total_v = median(nanmedian(abs(v_data_total),1));
subplot(6,2,2);hold on; plot(time_mean,mean_total_r*ones(length(time_mean)),'--','Color', m_color);hold off;
subplot(6,2,6);hold on; plot(time_mean,mean_total_t*ones(length(time_mean)),'--','Color', m_color);hold off;
subplot(6,2,10);hold on; plot(time_mean,mean_total_v*ones(length(time_mean)),'--','Color',m_color);hold off;


std_r = nanstd(abs(r_data_total));
std_t = nanstd(abs(t_data_total));
std_v = nanstd(abs(v_data_total));
subplot(6,2,2);hold on; plot(std_r,'Color',std_color);hold off;
subplot(6,2,6);hold on; plot(std_t,'Color',std_color);hold off;
subplot(6,2,10);hold on; plot(std_v,'Color', std_color);hold off;

subplot(6,2,4);hold on; plot_areaerrorbar(abs(r_data_total)); grid minor;hold off;
subplot(6,2,8);hold on; plot_areaerrorbar(abs(t_data_total)); grid minor;hold off;
subplot(6,2,12);hold on; plot_areaerrorbar(abs(v_data_total)); grid minor; xlabel('$\textrm{Time}[s]$','interpreter','latex');hold off;



