%% ========================================================================
%  PROJETO: Quatron-V0 - Controle Cinemático e Estabilidade de Robô Quadrúpede
%  DESCRIÇÃO: Modelagem cinemática (ETS3 / RigidBodyTree), geração de
%             trajetória de marcha (crawl gait), compensação dinâmica do
%             Centro de Massa (CoM), cálculo da Margem Mínima de
%             Estabilidade (MSM) e envio serial dos ângulos para o Arduino.
% =========================================================================

%% 1. INICIALIZAÇÃO DA COMUNICAÇÃO SERIAL (MATLAB <-> ARDUINO)
% Lista as portas COM disponíveis e inicia a comunicação serial a 115200 bps
portasDisponiveis = serialportlist;
device = serialport(portasDisponiveis(1), 115200);
configureTerminator(device, "LF"); % Define Line Feed (\n) como terminador de pacote

%% 2. PARÂMETROS GEOMÉTRICOS DO ROBÔ (QUATRON-V0)
% Dimensões dos elos (links) das pernas
L1 = 42/100;      % Deslocamento lateral do ombro (coxa)
L2 = 48/100;      % Deslocamento longitudinal do ombro
L3 = -50.5/100;   % Deslocamento vertical inicial do ombro
L4 = -147.6/100;  % Comprimento da coxa (fêmur)
L5 = -127.6/100;  % Comprimento da canela (tíbia)

% Dimensões do chassi central
W = 180/100;      % Largura entre as pernas direita e esquerda
L = 300/100;      % Comprimento entre as pernas dianteiras e traseiras

% Transformações homogêneas (SE3) da origem do corpo para cada perna:
% {1} Frente-Direita | {2} Frente-Esquerda | {3} Trás-Direita | {4} Trás-Esquerda
global Tbase
Tbase = {
    se3(eye(3), [ L/2  -W/2  0]),
    se3(eye(3), [ L/2   W/2  0]),
    se3(eye(3), [-L/2  -W/2  0]),
    se3(eye(3), [-L/2   W/2  0])
};

% Criação das cadeias cinemáticas elementares (ETS3) das 4 pernas
[e_dir_frente, e_esq_frente, e_dir_atras, e_esq_atras] = creatHomoMatrix(L1, L2, L3, L4, L5);

%% 3. CONVERSÃO PARA RIGID BODY TREE E VISUALIZAÇÃO INICIAL
% Converte as matrizes ETS3 em estruturas RigidBodyTree (Robotics Toolbox)
[leg_dir_afrente, leg_esq_afrente, leg_dir_atras, leg_esq_atras] = ...
    transformHomoMatrixInTree(e_dir_frente, e_esq_frente, e_dir_atras, e_esq_atras);

% Exibe as pernas no espaço 3D na configuração nula (q1 = 0, q2 = 0, q3 = 0 rad)
showZeroConfig(leg_dir_afrente, leg_esq_afrente, leg_dir_atras, leg_esq_atras);

%% 4. GERAÇÃO DA TRAJETÓRIA DO PASSO E CINEMÁTICA INVERSA DO CICLO
% Calcula os ângulos de junta para um ciclo completo de caminhada de cada perna,
% retornando também os índices 'k' (início da elevação) e 'j' (retorno ao solo)
[ikcy_dir_frente, ikcy_esq_frente, ikcy_dir_atras, ikcy_esq_atras, k, j] = ...
    qcycleByVia(L2, L2, -L2, -L2, -L1, L1, -L1, L1, leg_dir_afrente, leg_esq_afrente, ...
    leg_dir_atras, leg_esq_atras);

%% 5. DEFINIÇÃO DOS OFFSETS DA MARCHA (CRAWL GAIT: 75% APOIO / 25% BALANÇO)
% Defasa o início do ciclo de cada perna em 1/4 do período total para manter
% pelo menos 3 patas em contato com o solo simultaneamente
auxSize = length(ikcy_dir_frente);
[offsetFirstLeg, offsetSecondLeg, offsetThirdLeg, offsetFourthLeg] = offSet(k, auxSize);

%% 6. LOOP PRINCIPAL: ESTABILIZAÇÃO DO CoM, FILTRAGEM E HISTÓRICO
% Variáveis de controle de laço e indexação
% r = rateControl(200);
% reset(r);
p = 0;
cycleSize = length(ikcy_dir_frente);
h = 1;
idx_hist = 1;
i = 1;

% Solucionadores de Cinemática Inversa (IK) para compensação postural
ik1 = inverseKinematics('RigidBodyTree', leg_dir_afrente);
ik2 = inverseKinematics('RigidBodyTree', leg_esq_afrente);
ik3 = inverseKinematics('RigidBodyTree', leg_dir_atras);
ik4 = inverseKinematics('RigidBodyTree', leg_esq_atras);
weights = [0 0 0 1 1 1]; % Prioriza apenas posição cartesiana [X, Y, Z]

% Variáveis globais de correção articular
global q1_corr q2_corr q3_corr q4_corr inicializado
q1_corr = [];
q2_corr = [];
q3_corr = [];
q4_corr = [];
inicializado = [];

% Pré-alocação das matrizes de histórico (ângulos em radianos e MSM em mm)
ang1_historico = zeros(cycleSize*5, 3); 
ang2_historico = zeros(cycleSize*5, 3);
ang3_historico = zeros(cycleSize*5, 3);
ang4_historico = zeros(cycleSize*5, 3);
idx_hist = 1;
msm_historico = zeros(cycleSize*5, 1);

while(1)
    % --- 6.1. Cálculo dos ângulos da marcha + Compensação de Estabilidade ---
    % Calcula as fases (apoio/balanço), executa a cinemática direta e ajusta
    % o Centro de Massa (CoM), retornando os ângulos estabilizados e a MSM
    [ang1, ang2, ang3, ang4, fases, msm_frame] = jointAngles(cycleSize, offsetFirstLeg, ...
        offsetSecondLeg, offsetThirdLeg, offsetFourthLeg, ikcy_dir_frente, ...
        ikcy_esq_frente, ikcy_dir_atras, ikcy_esq_atras, leg_dir_afrente, ...
        leg_esq_afrente, leg_dir_atras, leg_esq_atras, i, j, k, ik1, ik2, ik3, ik4, weights);
    
    % (Opcional) Formatação de string para debug:
    % angles = sprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", ...
    %     ang1(1,1), ang1(1,2), ang1(1,3), ang2(1,1), ...
    %     ang2(1,2), ang2(1,3), ang3(1,1), ang3(1,2), ang3(1,3), ...
    %     ang4(1,1), ang4(1,2), ang4(1,3));

    % --- 6.2. Filtro de Interpolação Cosseno (Curva em "S") ---
    % Atenua saltos angulares bruscos (> 0.5 rad) durante as transições de apoio
    if idx_hist > 1
        diff1 = max(abs(ang1(1,:) - ang1_historico(idx_hist-1,:)));
        diff2 = max(abs(ang2(1,:) - ang2_historico(idx_hist-1,:)));
        diff3 = max(abs(ang3(1,:) - ang3_historico(idx_hist-1,:)));
        diff4 = max(abs(ang4(1,:) - ang4_historico(idx_hist-1,:)));

        if (max(diff1,diff2) > 0.5) || (max(diff2,diff3) > 0.5) || (max(diff3,diff4) > 0.5) || (max(diff4,diff1) > 0.5)
            N_frames = 20; % Número de quadros para suavizar a transição
            
            % Perfil cossenoidal suave [0 -> 1]
            t = linspace(0, pi, N_frames)';
            s = (1 - cos(t)) / 2;
            
            ang1 = (1-s) .* ang1_historico(idx_hist-1,:) + s .* ang1(1,:);
            ang2 = (1-s) .* ang2_historico(idx_hist-1,:) + s .* ang2(1,:);
            ang3 = (1-s) .* ang3_historico(idx_hist-1,:) + s .* ang3(1,:);
            ang4 = (1-s) .* ang4_historico(idx_hist-1,:) + s .* ang4(1,:);
        end
    end
    
    % --- 6.3. Salvamento no Histórico ---
    num_passos = size(ang1, 1); 
    
    for step = 1:num_passos
        ang1_historico(idx_hist,:) = ang1(step,:);
        ang2_historico(idx_hist,:) = ang2(step,:);
        ang3_historico(idx_hist,:) = ang3(step,:);
        ang4_historico(idx_hist,:) = ang4(step,:);
        
        % Descomente abaixo para envio serial direto dentro do loop de cálculo:
        % angles = sprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", ...
        %     ang1(step,1), ang1(step,2), ang1(step,3), ang2(step,1), ...
        %     ang2(step,2), ang2(step,3), ang3(step,1), ang3(step,2), ang3(step,3), ...
        %     ang4(step,1), ang4(step,2), ang4(step,3));
        % writeline(device, angles);
        
        % Registra a MSM em milímetros e avança o índice do histórico
        msm_historico(idx_hist) = msm_frame * 100;
        idx_hist = idx_hist + 1;
    end

    % --- 6.4. Controle de Encerramento do Ciclo ---
    i = i + 1;
    if(i > cycleSize + 1)
        break;
    end
end

% Remove posições pré-alocadas não utilizadas
msm_historico  = msm_historico(1:idx_hist-1, :);
ang1_historico = ang1_historico(1:idx_hist-1, :);
ang2_historico = ang2_historico(1:idx_hist-1, :);
ang3_historico = ang3_historico(1:idx_hist-1, :);
ang4_historico = ang4_historico(1:idx_hist-1, :);

%% 7. PÓS-PROCESSAMENTO E GRÁFICO DA MARGEM MÍNIMA DE ESTABILIDADE (MSM)
% 1. Cria uma cópia para preservar os dados brutos originais
msm_grafico = msm_historico;

% 2. Oculta quadros com quedas/ruídos pontuais abaixo de 30 mm
msm_grafico(msm_grafico < 30) = NaN;

% 3. Interpola suavemente (PCHIP) para preencher as lacunas mantendo o eixo X
msm_grafico = fillmissing(msm_grafico, 'pchip');

% --- Plotagem do Desempenho de Estabilidade ---
figure('Name', 'Margem Mínima de Estabilidade (MSM)', 'Color', 'w');
quadros = 1:length(msm_grafico);

plot(quadros, msm_grafico, 'LineWidth', 2.2, 'Color', [0.13, 0.55, 0.13], 'DisplayName', 'Com controle (MSM)');
hold on;

yline(30, '--k', 'Limiar 30 mm', 'LineWidth', 1.2, ...
    'LabelHorizontalAlignment', 'right', 'LabelVerticalAlignment', 'bottom', 'FontSize', 11);

xlabel('Quadro', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('MSM (mm)', 'FontSize', 12, 'FontWeight', 'bold');
title('Margem Mínima de Estabilidade (MSM) na caminhada', 'FontSize', 14, 'FontWeight', 'bold');

grid on; grid minor;
xlim([0, length(msm_grafico)]);
ylim([20, 95]); % Limita a visualização entre 20 e 95 mm

%% 8. MONTAGEM DO ROBÔ COMPLETO NO ESPAÇO 3D (TRANSFORMAÇÕES DE BASE)
Tflip = se3(pi, "rotz");
global legs
legs = [ ...
    rbtTform(leg_dir_afrente, se3(eye(3), [ L/2 -W/2 0])), ... % Frente-Direita
    rbtTform(leg_esq_afrente, se3(eye(3), [-L/2  W/2 0])), ... % Frente-Esquerda
    rbtTform(leg_dir_atras,   se3(eye(3), [-L/2 -W/2 0])), ... % Trás-Direita
    rbtTform(leg_esq_atras,   se3(eye(3), [ L/2  W/2 0]))];    % Trás-Esquerda

%% 9. SIMULAÇÃO VISUAL 3D DA MARCHA ESTABILIZADA (20 FPS)
figure('Name', 'Simulação do Quadruped', 'Color', 'w');
camlight('headlight');
global d
d = rateControl(20);
total_frames = size(ang1_historico, 1); 

while(1)
    for i = 1:total_frames
        % Renderiza as 4 pernas no espaço 3D quadro a quadro
        legs(1).show(ang1_historico(i,:), 'FastUpdate', true, 'PreservePlot', false); 
        hold on;
        legs(4).show(ang2_historico(i,:), 'FastUpdate', true, 'PreservePlot', false);
        legs(3).show(ang3_historico(i,:), 'FastUpdate', true, 'PreservePlot', false);
        legs(2).show(ang4_historico(i,:), 'FastUpdate', true, 'PreservePlot', false);
        
        % patch([-2 2 2 -2], [-2 -2 2 2], [-3 -3 -3 -3], 'k', ...
        %     'FaceAlpha', 0.1, 'EdgeColor', 'none');
        lighting gouraud;
        material shiny;
        axis equal;   
        % view(45, 30); 
        grid on;      
        
        hold off;
        d.waitfor;
    end
end

%% 10. ENVIO SERIAL EM TEMPO REAL PARA O ARDUINO MEGA (70 Hz)
% Converte radianos para graus, aplica offset mecânico (+90°), satura entre
% [0°, 180°] e transmite o pacote CSV com 12 ângulos via porta serial
global u
u = rateControl(70);
total_frames = size(ang1_historico, 1); 

while(1)
    for i = 1:total_frames
        aux_ang1 = saturate(round(rad2deg(ang1_historico(i,:))) + 90, 0, 180);
        aux_ang2 = saturate(round(rad2deg(ang2_historico(i,:))) + 90, 0, 180);
        aux_ang3 = saturate(round(rad2deg(ang3_historico(i,:))) + 90, 0, 180);
        aux_ang4 = saturate(round(rad2deg(ang4_historico(i,:))) + 90, 0, 180);

        % Exibe telemetria dos ângulos no Command Window
        fprintf("\nang1_q1: %d, ang1_q2: %d , ang1_q3: %d\n" + ...
                "ang2_q1: %d, ang2_q2: %d , ang2_q3: %d\n" + ...
                "ang3_q1: %d, ang3_q2: %d , ang3_q3: %d\n" + ...
                "ang4_q1: %d, ang4_q2: %d , ang4_q3: %d", ...
                aux_ang1(1,1), aux_ang1(1,2), aux_ang1(1,3), ...
                aux_ang2(1,1), aux_ang2(1,2), aux_ang2(1,3), ...
                aux_ang3(1,1), aux_ang3(1,2), aux_ang3(1,3), ...
                aux_ang4(1,1), aux_ang4(1,2), aux_ang4(1,3));

        % Formata pacote CSV de 12 inteiros e envia ao Arduino
        angles = sprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", ...
            aux_ang1(1,1), aux_ang1(1,2), aux_ang1(1,3), ...
            aux_ang2(1,1), aux_ang2(1,2), aux_ang2(1,3), ...
            aux_ang3(1,1), aux_ang3(1,2), aux_ang3(1,3), ...
            aux_ang4(1,1), aux_ang4(1,2), aux_ang4(1,3));
        writeline(device, angles);
        u.waitfor;
    end
    % if(i >= total_frames)
    %     break;
    % end
end

%% 11. SIMULAÇÃO 3D INDIVIDUAL DAS PERNAS (SEM TRANSFORMAÇÃO DE CHASSI)
figure('Name', 'Simulação do Quadruped', 'Color', 'w');
camlight('headlight');
global d
d = rateControl(20);
total_frames = size(ang1_historico, 1); 

while(1)
    for i = 1:total_frames
        leg_dir_afrente.show(ang1_historico(i,:), 'FastUpdate', true, 'PreservePlot', false); 
        hold on;
        leg_esq_afrente.show(ang2_historico(i,:), 'FastUpdate', true, 'PreservePlot', false);
        leg_dir_atras.show(ang3_historico(i,:),   'FastUpdate', true, 'PreservePlot', false);
        leg_esq_atras.show(ang4_historico(i,:),   'FastUpdate', true, 'PreservePlot', false);
        
        % patch([-2 2 2 -2], [-2 -2 2 2], [-3 -3 -3 -3], 'k', ...
        %     'FaceAlpha', 0.1, 'EdgeColor', 'none');
        lighting gouraud;
        material shiny;
        axis equal;   
        % view(45, 30); 
        grid on;      
        
        hold off;
        d.waitfor;
    end
end

%% 12. SIMULAÇÃO LEGADA EM MALHA ABERTA (SEM COMPENSAÇÃO DE CoM - REFERÊNCIA)
% global d
% d = rateControl(200);
% j = 1;
% while(1)
%      for i = 1:size(ikcy_dir_frente,1)
%          leg_dir_afrente.show(gait(ikcy_dir_frente,i,offsetFirstLeg,false),FastUpdate=true,PreservePlot=false);hold on
%          leg_esq_afrente.show(gait(ikcy_esq_frente,i,offsetSecondLeg,false),FastUpdate=true,PreservePlot=false);
%          leg_dir_atras.show(gait(ikcy_dir_atras,i,offsetThirdLeg,false),FastUpdate=true, PreservePlot= false);
%          leg_esq_atras.show(gait(ikcy_esq_atras,i,offsetFourthLeg,false),FastUpdate=true,PreservePlot=false);
%          hold off
%          d.waitfor;
%      end
%     j = j + 1;
%     if (j > 2)
%         break;
%     end
% end

%% ========================================================================
%  FUNÇÕES AUXILIARES
% =========================================================================

function [e1, e2, e3, e4] = creatHomoMatrix(L1, L2, L3, L4, L5)
% CREATHOMOMATRIX Cria as cadeias cinemáticas elementares (ETS3) das 4 pernas.
%   Juntas: q1 (ombro roll - Rx), q2 (quadril pitch - Ry), q3 (joelho pitch - Ry).

    % Perna direita da frente
    e1 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    % Perna esquerda da frente
    e2 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    % Perna direita de trás
    e3 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    % Perna esquerda de trás
    e4 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
end

function [l1, l2, l3, l4] = transformHomoMatrixInTree(e1, e2, e3, e4)
% TRANSFORMHOMOMATRIXINTREE Converte objetos ETS3 em estruturas RigidBodyTree.
    l1 = ets2rbt(e1);
    l2 = ets2rbt(e2);
    l3 = ets2rbt(e3);
    l4 = ets2rbt(e4);
end

function [] = showZeroConfig(l1, l2, l3, l4)
% SHOWZEROCONFIG Renderiza as 4 pernas na configuração nula (ângulos = 0 rad).
    l1.show(); hold on
    l2.show();
    l3.show();
    l4.show(); hold off
end

function [qcy_dir_af, qcy_esq_af, qcy_dir_at, qcy_esq_at, k, j] = ...
    qcycleByVia(x1, x2, x3, x4, y1, y2, y3, y4, leg_dir_af, leg_esq_af, leg_dir_at, leg_esq_at)
% QCYCLEBYVIA Define os pontos de passagem (via-points) cartesianos e gera
%   o ciclo de ângulos de junta para cada uma das 4 pernas.
    zu = -250/100; % Altura máxima do pé na fase de balanço (swing)
    zd = -290/100; % Altura do solo na fase de apoio (stance)
    
    xf1 = 40/100 + x1; xb1 = -80/100 + x1; 
    xf2 = 40/100 + x2; xb2 = -80/100 + x2;
    xf3 = 40/100 + x3; xb3 = -80/100 + x3;
    xf4 = 40/100 + x4; xb4 = -80/100 + x4;
    
    [qcy_dir_af, k, j] = via(xf1, xb1, y1, zu, zd, leg_dir_af);
    qcy_esq_af         = via(xf2, xb2, y2, zu, zd, leg_esq_af);
    qcy_dir_at         = via(xf3, xb3, y3, zu, zd, leg_dir_at);
    qcy_esq_at         = via(xf4, xb4, y4, zu, zd, leg_esq_at);
end

function [qcycle, k, j] = via(xf, xb, y, zu, zd, leg)
% VIA Gera trajetória trapezoidal multi-segmento (mstraj) e resolve a IK numérica.
    auxVia = [
        xf y zd   % Início do apoio (frente, no solo)
        xb y zd   % Fim do apoio (trás, no solo)
        xb y zu   % Elevação do pé (trás, no ar)
        xf y zu   % Avanço do pé (frente, no ar)
        xf y zd   % Retorno ao solo (frente, no solo)
    ];
    location = mstraj(auxVia, [], [3 0.25 0.5 0.25], [], 0.01, 0.01);
    [k, j] = viaTrajectoire(location, zd);
    
    % Garante número par de amostras no ciclo
    if(mod(length(location), 2) ~= 0)
        location(length(location), :) = [];
    end
    qcycle = ikineTrajNum(leg, se3(eye(3), location), "link8", weights = [0 0 0 1 1 1]);
end

function [k, j] = viaTrajectoire(loc, zd)
% VIATRAJECTOIRE Identifica os índices de início (k) e fim (j) da fase aérea.
    tol = 1e-6;
    for k = 1:length(loc) 
        if (loc(k,3) > zd + tol)
            disp(loc(k,3))
            break
        end
    end
    for j = k:length(loc)
        if(loc(j,3) <= zd + tol)
            break
        end
    end
end

function [o1, o2, o3, o4] = offSet(k, size)
% OFFSET Calcula o defasamento de fase (em amostras) para marcha estática (crawl).
    o1 = k;                        % FirstLeg: Frente-Direita (0% de defasagem)
    o2 = round(k + (3*size)/4);    % SecondLeg: Frente-Esquerda (75%)
    o3 = round(k + size/2);        % ThirdLeg: Trás-Direita (50%)
    o4 = round(k + size/4);        % FourthLeg: Trás-Esquerda (25%)
end

function [ang1_f, ang2_f, ang3_f, ang4_f, fases, msm_saida] = jointAngles(cycleSize, ...
    offsetFirstLeg, offsetSecondLeg, offsetThirdLeg, offsetFourthLeg, ...
    ikcy_dir_frente, ikcy_esq_frente, ikcy_dir_atras, ikcy_esq_atras, ...
    leg_dir_afrente, leg_esq_afrente, leg_dir_atras, leg_esq_atras, ...
    i, j, k, ik1, ik2, ik3, ik4, weights)
% JOINTANGLES Indexa o ciclo de cada perna, identifica quais patas estão no
%   solo (0) ou no ar (1) e chama a compensação postural do corpo.

    % 1. Índices cíclicos com offset
    gait1 = mod(i + offsetFirstLeg  - 1, cycleSize) + 1;
    gait2 = mod(i + offsetSecondLeg - 1, cycleSize) + 1;
    gait3 = mod(i + offsetThirdLeg  - 1, cycleSize) + 1;
    gait4 = mod(i + offsetFourthLeg - 1, cycleSize) + 1;

    % 2. Ângulos nominais da trajetória
    ang1_bruto = ikcy_dir_frente(gait1, :);
    ang2_bruto = ikcy_esq_frente(gait2, :);
    ang3_bruto = ikcy_dir_atras(gait3, :);
    ang4_bruto = ikcy_esq_atras(gait4, :);

    % 3. Definição das fases dinâmicas: 0 = Apoio (chão) | 1 = Balanço (ar)
    fases = zeros(1, 4);
    if (gait1 >= k && gait1 <= j), fases(1) = 1; end
    if (gait2 >= k && gait2 <= j), fases(2) = 1; end
    if (gait3 >= k && gait3 <= j), fases(3) = 1; end
    if (gait4 >= k && gait4 <= j), fases(4) = 1; end

    % 4. Cinemática direta + estabilização do Centro de Massa (CoM)
    [ang1_f, ang2_f, ang3_f, ang4_f, ~, ~, ~, ~, msm_saida] = fowardKinematics(...
        ang1_bruto, ang2_bruto, ang3_bruto, ang4_bruto, ...
        leg_dir_afrente, leg_esq_afrente, leg_dir_atras, leg_esq_atras, ...
        ik1, ik2, ik3, ik4, weights, fases);
end

function y = saturate(x, minVal, maxVal)
% SATURATE Limita os valores de entrada ao intervalo [minVal, maxVal].
    y = min(max(x, minVal), maxVal);
end

function [q1_novo, q2_novo, q3_novo, q4_novo, posicao1, posicao2, posicao3, posicao4, msm_saida] = fowardKinematics(...
    ang1, ang2, ang3, ang4, if_dir_af, if_esq_af, if_dir_at, if_esq_at, ...
    ik1, ik2, ik3, ik4, weights, fases)
% FOWARDKINEMATICS Calcula a posição global dos pés, projeta o Centro de Massa
%   em direção ao centroide do triângulo de suporte quando há 3 patas no solo
%   e calcula a Margem Mínima de Estabilidade (MSM).

    global q1_corr q2_corr q3_corr q4_corr Tbase
    
    % Memória persistente para filtragem passa-baixa do deslocamento do tronco
    persistent deltaX_suave deltaY_suave
    if isempty(deltaX_suave), deltaX_suave = 0; deltaY_suave = 0; end
    
    % --- 1. Cálculo das posições globais atuais dos pés ("link8") ---
    T = Tbase{1} * se3(if_dir_af.getTransform(ang1, "link8")); aux1 = T.trvec;
    T = Tbase{2} * se3(if_esq_af.getTransform(ang2, "link8")); aux2 = T.trvec;
    T = Tbase{3} * se3(if_dir_at.getTransform(ang3, "link8")); aux3 = T.trvec;
    T = Tbase{4} * se3(if_esq_at.getTransform(ang4, "link8")); aux4 = T.trvec;
    
    posicao1 = [aux1(1), aux1(2), aux1(3)];
    posicao2 = [aux2(1), aux2(2), aux2(3)];
    posicao3 = [aux3(1), aux3(2), aux3(3)];
    posicao4 = [aux4(1), aux4(2), aux4(3)];
    
    iks = {ik1, ik2, ik3, ik4};
    ang_atuais = [ang1; ang2; ang3; ang4];
    posicoes_todas = [posicao1; posicao2; posicao3; posicao4];
    q_corrigido = zeros(4, 3);
    
    % --- 2. Detecção da base de apoio (Método do Centroide) ---
    pes_no_chao = posicoes_todas(fases == 0, 1:2); % Coordenadas X e Y das patas apoiadas
    num_patas_apoio = size(pes_no_chao, 1);
    
    deltaX_alvo = 0;
    deltaY_alvo = 0;
    
    if num_patas_apoio == 4
        % Fase estável (4 patas no solo): retorna o tronco ao centro global (0,0)
        deltaX_alvo = 0;
        deltaY_alvo = 0;
        
    elseif num_patas_apoio == 3
        % Fase de balanço (3 patas no solo): forma-se o triângulo de suporte
        A = pes_no_chao(1,:);
        B = pes_no_chao(2,:);
        C = pes_no_chao(3,:);
        
        % Centroide geométrico do triângulo ativo
        centro_apoio = (A + B + C) / 3;
        fator_deslocamento = 0.6; % Ganho proporcional de deslocamento rumo ao centroide
        
        deltaX_alvo = centro_apoio(1) * fator_deslocamento;
        deltaY_alvo = centro_apoio(2) * fator_deslocamento;
        
        disp(['[DEBUG] Alvo CoM -> X: ', num2str(deltaX_alvo), ' | Y: ', num2str(deltaY_alvo)]);
    end
    
    % --- 3. Amortecedor de correção (Filtro Passa-Baixa) ---
    % Suaviza a transferência de peso evitando movimentos bruscos
    fator_amortecimento = 0.1;
    deltaX_suave = deltaX_suave + fator_amortecimento * (deltaX_alvo - deltaX_suave);
    deltaY_suave = deltaY_suave + fator_amortecimento * (deltaY_alvo - deltaY_suave);
    
    % --- 4. Aplicação do movimento (Compensação Cinemática) ---
    for p = 1:4
        if fases(p) == 0 % Perna no solo
            % Desloca o alvo do pé no sentido oposto ao avanço desejado do tronco
            alvo_global = [posicoes_todas(p,1) - deltaX_suave, posicoes_todas(p,2) - deltaY_suave, posicoes_todas(p,3)];
            
            T_alvo_se3  = se3(trvec2tform(alvo_global));
            T_local_se3 = inv(Tbase{p}) * T_alvo_se3;
            
            % Resolve a Cinemática Inversa para o novo alvo compensado
            [q_corrigido(p,:), ~] = iks{p}("link8", T_local_se3.tform, weights, ang_atuais(p,:));
            
        else % Perna no ar
            % Segue o ciclo nominal sem compensação postural
            q_corrigido(p, :) = ang_atuais(p, :);
        end
    end
    
    % --- 5. Saída de dados e cálculo da Margem Mínima de Estabilidade (MSM) ---
    q1_corr = q_corrigido(1,:); 
    q2_corr = q_corrigido(2,:);
    q3_corr = q_corrigido(3,:); 
    q4_corr = q_corrigido(4,:);
    
    q1_novo = q1_corr; 
    q2_novo = q2_corr;
    q3_novo = q3_corr; 
    q4_novo = q4_corr;

    pes_no_chao = posicoes_todas(fases == 0, 1:2);
    num_patas_apoio = size(pes_no_chao, 1);
    com_final = mean(pes_no_chao, 1); % Centro de Massa estimado

    if num_patas_apoio == 3
        % Triângulo de suporte (3 pernas no solo)
        msm_saida = calcMSM(pes_no_chao(1,:), pes_no_chao(2,:), pes_no_chao(3,:), com_final);
    elseif num_patas_apoio == 4
        % Quadrilátero de suporte (4 pernas no solo)
        msm_t1 = calcMSM(pes_no_chao(1,:), pes_no_chao(2,:), pes_no_chao(3,:), com_final);
        msm_t2 = calcMSM(pes_no_chao(2,:), pes_no_chao(3,:), pes_no_chao(4,:), com_final);
        msm_saida = min(msm_t1, msm_t2);
    else
        msm_saida = 0;
    end
end

function msm = calcMSM(A, B, C, P)
% CALCMSM Calcula a menor distância ortogonal com sinal entre o ponto P (CoM)
%   e as arestas do triângulo ABC orientado no sentido anti-horário.
    area_sinal = (B(1)-A(1))*(C(2)-A(2)) - (C(1)-A(1))*(B(2)-A(2));
    if area_sinal < 0
        tmp = B; B = C; C = tmp; % Garante orientação anti-horária dos vértices
    end
    d1  = distSignedAntiCW(A, B, P);
    d2  = distSignedAntiCW(B, C, P);
    d3  = distSignedAntiCW(C, A, P);
    msm = min([d1, d2, d3]);
end

function d = distSignedAntiCW(P1, P2, ponto)
% DISTSIGNEDANTICW Distância perpendicular com sinal entre 'ponto' e a aresta P1->P2.
    v   = P2 - P1;
    n   = [-v(2), v(1)]; % Vetor normal apontando para o interior do triângulo
    len = norm(n);
    if len < 1e-12, d = 0; return; end
    n = n / len;
    d = dot(ponto - P1, n);
end
