L1 = 36.75/100;
L2 = 45.75/100;
L3 = -50.5/100;
L4 = -153.5/100;
L5 = -130.5/100;

W = 150/100;
L = 250/100;

global Tbase
Tbase = {
    se3(eye(3), [ L/2  -W/2  0]),
    se3(eye(3), [ L/2   W/2  0]),
    se3(eye(3), [-L/2  -W/2  0]),
    se3(eye(3), [-L/2   W/2  0])
    };


[e_dir_frente,e_esq_frente,e_dir_atras,e_esq_atras] = creatHomoMatrix(L1,L2,L3,L4,L5);
%%
%Transformando as matrizes em corpos rigídos com formato/estrutura de
%arvore(Rigid Body Tree Structer).
[leg_dir_afrente,leg_esq_afrente,leg_dir_atras,leg_esq_atras] = transformHomoMatrixInTree(e_dir_frente,e_esq_frente,e_dir_atras,e_esq_atras);
%%
%Mostrando como estão as pernas no espaço 3D, quando todos os angulos das 
%juntas q1,q2 e q3 em radianos estão em 0. Conhecido como zero
%configuration
showZeroConfig(leg_dir_afrente,leg_esq_afrente,leg_dir_atras,leg_esq_atras);
%%
%qcycle1 e qcycle2 (Andar para frente com a perna direita e esquerda)
[ikcy_dir_frente,ikcy_esq_frente,ikcy_dir_atras,ikcy_esq_atras,k,j] =... 
    qcycleByVia(L2,L2,-L2,-L2,-L1,L1,-L1,L1,leg_dir_afrente,leg_esq_afrente, ...
    leg_dir_atras,leg_esq_atras);
%%
%Definindo os offsets com base no valor k, que determina o momento de
%subida no plano zx ou zy da trajetória.
auxSize = length(ikcy_dir_frente);
[offsetFirstLeg,offsetSecondLeg,offsetThirdLeg,offsetFourthLeg] = offSet(k,auxSize);
%%
%Obtendo os angulos das juntas de cada perna com base no ciclo de cada uma
%respeitando os offset que mantém o padrão de 25% no ar e 75% no chão
% usando a função gait para definir quando cada perna começa a se mexer. Após
%isto, enviando serialmente os valores dos angulos já em graus para o
%arduino com um ratecontrol de 50Hz, pois os servos motores tem uma freq de
%50hz.
%r = rateControl(200);
%reset(r);
p = 0;
cycleSize = length(ikcy_dir_frente);
h = 1;
idx_hist = 1;
i = 1;

% Objetos de Cinemática Inversa (Essenciais, mantenha)
ik1 = inverseKinematics('RigidBodyTree', leg_dir_afrente);
ik2 = inverseKinematics('RigidBodyTree', leg_esq_afrente);
ik3 = inverseKinematics('RigidBodyTree', leg_dir_atras);
ik4 = inverseKinematics('RigidBodyTree', leg_esq_atras);
weights = [0 0 0 1 1 1];

% Globais de correção (mantidas, mas SEM o aux)
global q1_corr q2_corr q3_corr q4_corr inicializado
q1_corr = [];
q2_corr = [];
q3_corr = [];
q4_corr = [];
inicializado = [];

% Inicialização das matrizes do seu Filtro Cosseno (Essenciais, mantenha)
ang1_historico = zeros(cycleSize*5,3); 
ang2_historico = zeros(cycleSize*5,3);
ang3_historico = zeros(cycleSize*5,3);
ang4_historico = zeros(cycleSize*5,3);
idx_hist = 1; % Garantindo que inicie no índice correto

while(1)
    
    % --- 1. CHAMADA ATUALIZADA (Sem o 'aux') ---
    % O jointAngles agora calcula a caminhada, descobre as fases, 
    % roda a fowardKinematics internamente e devolve os ângulos estabilizados
    [ang1, ang2, ang3, ang4, fases] = jointAngles(cycleSize, offsetFirstLeg,...
        offsetSecondLeg, offsetThirdLeg, offsetFourthLeg, ikcy_dir_frente, ...
        ikcy_esq_frente, ikcy_dir_atras, ikcy_esq_atras, leg_dir_afrente, ...
        leg_esq_afrente, leg_dir_atras, leg_esq_atras, i, j, k, ik1, ik2, ik3, ik4, weights);
    
    % (Opcional) formatar para debug:
    % angles = sprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"...
    % ,ang1(1,1),ang1(1,2),ang1(1,3),ang2(1,1),...
    % ang2(1,2),ang2(1,3),ang3(1,1),ang3(1,2),ang3(1,3),ang4(1,1),ang4...
    % (1,2),ang4(1,3));

    % --- 2. SEU FILTRO DE INTERPOLAÇÃO COSSENO (INTACTO) ---
    if idx_hist > 1
        diff1 = max(abs(ang1(1,:) - ang1_historico(idx_hist-1,:)));
        diff2 = max(abs(ang2(1,:) - ang2_historico(idx_hist-1,:)));
        diff3 = max(abs(ang3(1,:) - ang3_historico(idx_hist-1,:)));
        diff4 = max(abs(ang4(1,:) - ang4_historico(idx_hist-1,:)));

        % Se o cálculo de equilíbrio (ou passo) gerou um salto > 0.5 radianos
        if (max(diff1,diff2) > 0.5) || (max(diff2,diff3) > 0.5) || (max(diff3,diff4) > 0.5) || (max(diff4,diff1) > 0.5)
            N_frames = 20; % Quantidade de frames para suavizar a transição
            
            % Criando uma curva em "S" matemática nativa
            t = linspace(0, pi, N_frames)';
            s = (1 - cos(t)) / 2;
            
            ang1 = (1-s) .* ang1_historico(idx_hist-1,:) + s .* ang1(1,:);
            ang2 = (1-s) .* ang2_historico(idx_hist-1,:) + s .* ang2(1,:);
            ang3 = (1-s) .* ang3_historico(idx_hist-1,:) + s .* ang3(1,:);
            ang4 = (1-s) .* ang4_historico(idx_hist-1,:) + s .* ang4(1,:);
        end
    end
    
    % --- 3. SALVAMENTO E ENVIO ---
    num_passos = size(ang1, 1); 
    
    for step = 1:num_passos
        ang1_historico(idx_hist,:) = ang1(step,:);
        ang2_historico(idx_hist,:) = ang2(step,:);
        ang3_historico(idx_hist,:) = ang3(step,:);
        ang4_historico(idx_hist,:) = ang4(step,:);
        
        % Se for usar com o Arduino depois, descomente esta área:
        % angles = sprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"...
        % ,ang1(step,1),ang1(step,2),ang1(step,3),ang2(step,1),...
        % ang2(step,2),ang2(step,3),ang3(step,1),ang3(step,2),ang3(step,3),ang4(step,1),ang4...
        % (step,2),ang4(step,3));
        % writeline(device,angles);
        
        % Incrementa o contador do histórico
        idx_hist = idx_hist + 1;
    end

    % --- 4. CONTROLE DO CICLO ---
    i = i + 1;
    if(i > cycleSize+1)
        break;
    end
end

% Apara as matrizes para o tamanho real que foi utilizado
ang1_historico = ang1_historico(1:idx_hist-1, :);
ang2_historico = ang2_historico(1:idx_hist-1, :);
ang3_historico = ang3_historico(1:idx_hist-1, :);
ang4_historico = ang4_historico(1:idx_hist-1, :);
%%
global u
u = rateControl(20);
total_frames = size(ang1_historico, 1); 
while(1)
    for i = 1:total_frames
        aux_ang1 = saturate(round(rad2deg(ang1_historico(i,:)))+90,0,180);% ang1 = saturate(round(rad2deg(q1_corrigido))+90,0,180);
        aux_ang2 = saturate(round(rad2deg(ang2_historico(i,:)))+90,0,180);
        aux_ang3 = saturate(round(rad2deg(ang3_historico(i,:)))+90,0,180);
        aux_ang4 = saturate(round(rad2deg(ang4_historico(i,:)))+90,0,180);
        % ang2 = saturate(round(rad2deg(q2_corrigido))+90,0,180);
        % ang3 = saturate(round(rad2deg(q3_corrigido))+90,0,180);
        % ang4 = saturate(round(rad2deg(q4_corrigido))+90,0,180);
         fprintf("\nang1_q1: %d, ang1_q2: %d , ang1_q3: %d\n" + ...
         "ang2_q1: %d, ang2_q2: %d , ang2_q3: %d\n" + ...
         "ang3_q1: %d, ang3_q2: %d , ang3_q3: %d\n" + ...
         "ang4_q1: %d, ang4_q2: %d , ang4_q3: %d",aux_ang1(1,1),aux_ang1(1,2), ...
             aux_ang1(1,3),aux_ang2(1,1),aux_ang2(1,2),aux_ang2(1,3),aux_ang3 ...
             (1,1),aux_ang3(1,2),aux_ang3(1,3),aux_ang4(1,1),aux_ang4(1,2),aux_ang4(1,3));

        angles = sprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"...
         ,aux_ang1(1,1),aux_ang1(1,2),aux_ang1(1,3),aux_ang2(step,1),...
         aux_ang2(step,2),aux_ang2(step,3),aux_ang3(step,1),aux_ang3(step,2), ...
         aux_ang3(step,3),aux_ang4(step,1),aux_ang4(step,2),aux_ang4(step,3));
        writeline(device,angles);
        u.waitfor;
    end
    if(i>=total_frames)
            break;
    end
end
%%
figure('Name', 'Simulação do Quadruped', 'Color', 'w');
camlight('headlight');
global d
d = rateControl(20);
total_frames = size(ang1_historico, 1); 

while(1)
    for i = 1:total_frames
        
        % Desenha os robôs
        leg_dir_afrente.show(ang1_historico(i,:), 'FastUpdate', true, 'PreservePlot', false); 
        hold on;
        leg_esq_afrente.show(ang2_historico(i,:), 'FastUpdate', true, 'PreservePlot', false);
        leg_dir_atras.show(ang3_historico(i,:), 'FastUpdate', true, 'PreservePlot', false);
        leg_esq_atras.show(ang4_historico(i,:), 'FastUpdate', true, 'PreservePlot', false);
        
        % patch([-2 2 2 -2], [-2 -2 2 2], [-3 -3 -3 -3], 'k', ...
        %     'FaceAlpha', 0.1, 'EdgeColor', 'none');
        lighting gouraud;
        material shiny;
        axis equal;   
        %view(45, 30); 
        grid on;      
        
        hold off;
        d.waitfor;
    end
end
%%
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
%% 
function [e1,e2,e3,e4] = creatHomoMatrix(L1,L2,L3,L4,L5)
    %Perna direita da frente.
    e1 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    %Perna esquerda da frente.
    e2 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    %Perna direita de trás.
    e3 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    %Perna esquerda de trás.
    e4 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    end

%%
function [l1,l2,l3,l4] = transformHomoMatrixInTree(e1,e2,e3,e4)
    l1 = ets2rbt(e1);
    l2 = ets2rbt(e2);
    l3 = ets2rbt(e3);
    l4 = ets2rbt(e4);
end

%%
function [] = showZeroConfig(l1,l2,l3,l4)
    l1.show();hold on
    l2.show();
    l3.show();
    l4.show();hold off
end
%%
function [qcy_dir_af,qcy_esq_af,qcy_dir_at,qcy_esq_at,k,j] = ...
    qcycleByVia(x1,x2,x3,x4,y1,y2,y3,y4,leg_dir_af,leg_esq_af,leg_dir_at, ...
    leg_esq_at)
    zu = -200/100; zd = -250/100;
    xf1 = 40/100+x1; xb1 = -80/100+x1; 
    xf2 = 40/100+x2; xb2 = -80/100+x2;
    xf3 = 40/100+x3; xb3 = -80/100+x3;
    xf4 = 40/100+x4; xb4 = -80/100+x4 ;
    [qcy_dir_af,k,j] = via(xf1,xb1,y1,zu,zd,leg_dir_af);
    qcy_esq_af = via(xf2,xb2,y2,zu,zd,leg_esq_af);
    qcy_dir_at = via(xf3,xb3,y3,zu,zd,leg_dir_at);
    qcy_esq_at = via(xf4,xb4,y4,zu,zd,leg_esq_at);
end
%%
function [qcycle,k,j] = via(xf,xb,y,zu,zd,leg)
    auxVia = [xf y zd
        xb y zd
        xb y zu
        xf y zu
        xf y zd
    ];
    location = mstraj(auxVia,[],[3 0.25 0.5 0.25],[],0.01,0.01);
    [k,j] = viaTrajectoire(location,zd);
    if(mod(length(location),2)~=0)
        location(length(location),:) = [];
    end
    qcycle = ikineTrajNum(leg,se3(eye(3),location),"link8",weights = [0 0 0 1 1 1]);
    
end
%%
function [k,j] = viaTrajectoire(loc,zd)
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
%%
function [o1,o2,o3,o4] = offSet(k,size)
    %FirstLeg é igual à primeiraPerna-frente
    o1 = k;
    %SecondLeg é igual à segundaPerna-frente
    o2 = round(k + (3*size)/4);
    %ThirdLeg é igual à primeiraPerna-Atras
    o3 = round(k + size/2);
    %FourthLeg é igual à segundaPerna-Atras
    o4 = round(k + size/4);
end
%%
function [ang1_f, ang2_f, ang3_f, ang4_f, fases] = jointAngles(cycleSize, ...
    offsetFirstLeg, offsetSecondLeg, offsetThirdLeg, offsetFourthLeg, ...
    ikcy_dir_frente, ikcy_esq_frente, ikcy_dir_atras, ikcy_esq_atras, ...
    leg_dir_afrente, leg_esq_afrente, leg_dir_atras, leg_esq_atras, ...
    i, j, k, ik1, ik2, ik3, ik4, weights)

    % --- 1. CÁLCULO DOS ÍNDICES DE GAIT (TRAJETÓRIA) ---
    % Mantive a lógica padrão de offsets (ajuste se a sua for ligeiramente diferente)
    gait1 = mod(i + offsetFirstLeg - 1, cycleSize) + 1;
    gait2 = mod(i + offsetSecondLeg - 1, cycleSize) + 1;
    gait3 = mod(i + offsetThirdLeg - 1, cycleSize) + 1;
    gait4 = mod(i + offsetFourthLeg - 1, cycleSize) + 1;

    % --- 2. EXTRAÇÃO DOS ÂNGULOS BRUTOS DO CICLO (Apenas a intenção de movimento) ---
    ang1_bruto = ikcy_dir_frente(gait1, :);
    ang2_bruto = ikcy_esq_frente(gait2, :);
    ang3_bruto = ikcy_dir_atras(gait3, :);
    ang4_bruto = ikcy_esq_atras(gait4, :);

    % --- 3. DEFINIÇÃO DAS FASES DINÂMICAS ---
    % Cria um vetor [0 0 0 0] onde 0 significa "Perna no Chão" (Apoio) e 1 "No Ar" (Balanço)
    fases = zeros(1,4);
    
    % Usa os seus limites 'k' e 'j' para saber quem está na fase aérea
    if (gait1 >= k && gait1 <= j), fases(1) = 1; end
    if (gait2 >= k && gait2 <= j), fases(2) = 1; end
    if (gait3 >= k && gait3 <= j), fases(3) = 1; end
    if (gait4 >= k && gait4 <= j), fases(4) = 1; end

    % --- 4. CHAMADA DA FOWARD KINEMATICS (Estabilização do Corpo) ---
    % Agora enviamos os ângulos brutos e as 'fases' para que a função trave as pernas certas
    [ang1_f, ang2_f, ang3_f, ang4_f, ~, ~, ~, ~] = fowardKinematics(...
        ang1_bruto, ang2_bruto, ang3_bruto, ang4_bruto, ...
        leg_dir_afrente, leg_esq_afrente, leg_dir_atras, leg_esq_atras, ...
        ik1, ik2, ik3, ik4, weights, fases);

    % A função já retorna ang1_f, ang2_f, etc., e o vetor 'fases',
    % que vão direto lá para o seu while(1) passar pelo filtro em S.
end
%%
function y = saturate(x,minVal,maxVal)
    y = min(max(x,minVal),maxVal);
end
%%
function [q1_novo, q2_novo, q3_novo, q4_novo, posicao1, posicao2, posicao3, posicao4] = fowardKinematics(...
    ang1, ang2, ang3, ang4, if_dir_af, if_esq_af, if_dir_at, if_esq_at, ...
    ik1, ik2, ik3, ik4, weights, fases)
    
    global q1_corr q2_corr q3_corr q4_corr Tbase
    
    % Filtro interno para o balanço do corpo não dar tranco no pé
    persistent deltaX_suave deltaY_suave
    if isempty(deltaX_suave), deltaX_suave = 0; deltaY_suave = 0; end
    
    % --- 1. CÁLCULO DAS POSIÇÕES GLOBAIS ATUAIS DOS PÉS ---
    T = Tbase{1} * se3(if_dir_af.getTransform(ang1, "link8"));  aux1 = T.trvec;
    T = Tbase{2} * se3(if_esq_af.getTransform(ang2, "link8"));  aux2 = T.trvec;
    T = Tbase{3} * se3(if_dir_at.getTransform(ang3, "link8"));  aux3 = T.trvec;
    T = Tbase{4} * se3(if_esq_at.getTransform(ang4, "link8"));  aux4 = T.trvec;
    
    posicao1 = [aux1(1), aux1(2), aux1(3)];
    posicao2 = [aux2(1), aux2(2), aux2(3)];
    posicao3 = [aux3(1), aux3(2), aux3(3)];
    posicao4 = [aux4(1), aux4(2), aux4(3)];
    
    iks = {ik1, ik2, ik3, ik4};
    ang_atuais = [ang1; ang2; ang3; ang4];
    posicoes_todas = [posicao1; posicao2; posicao3; posicao4];
    q_corrigido = zeros(4,3);
    
    % --- 2. DETECÇÃO DA BASE DE APOIO (MÉTODO DO CENTROIDE) ---
    pes_no_chao = posicoes_todas(fases == 0, 1:2); % Pega apenas X e Y das patas no chão
    num_patas_apoio = size(pes_no_chao, 1);
    
    deltaX_alvo = 0;
    deltaY_alvo = 0;
    
    if num_patas_apoio == 4
        % Fase estável: 4 patas no chão. O corpo volta para o centro global (0,0)
        deltaX_alvo = 0;
        deltaY_alvo = 0;
        
    elseif num_patas_apoio == 3
        % Fase de balanço: 1 perna no ar. Forma-se o triângulo de suporte.
        A = pes_no_chao(1,:);
        B = pes_no_chao(2,:);
        C = pes_no_chao(3,:);
        
        % O Centroide (média geométrica) é a zona de estabilidade máxima
        centro_apoio = (A + B + C) / 3;
        
        % Fator de deslocamento define o quanto do caminho até o centroide o robô vai percorrer
        % Tente valores entre 0.4 e 0.8 (começando com 0.6)
        fator_deslocamento = 0.6; 
        
        deltaX_alvo = centro_apoio(1) * fator_deslocamento;
        deltaY_alvo = centro_apoio(2) * fator_deslocamento;
        
        % PRINT PARA DEBUG (Pode comentar essa linha depois que ver funcionando)
        disp(['[DEBUG] Alvo CoM -> X: ', num2str(deltaX_alvo), ' | Y: ', num2str(deltaY_alvo)]);
    end
    
    % --- 3. AMORTECEDOR DE CORREÇÃO (Filtro Passa-Baixa) ---
    % Garante que a mudança para o novo alvo seja fluida e não um pulo repentino
    fator_amortecimento = 0.1;
    deltaX_suave = deltaX_suave + fator_amortecimento * (deltaX_alvo - deltaX_suave);
    deltaY_suave = deltaY_suave + fator_amortecimento * (deltaY_alvo - deltaY_suave);
    
    % --- 4. APLICAÇÃO DO MOVIMENTO (COMPENSAÇÃO CINEMÁTICA) ---
    for p = 1:4
        if fases(p) == 0 % PERNA NO CHÃO
            % Desloca a coordenada alvo no sentido oposto para o corpo ir na direção certa
            alvo_global = [posicoes_todas(p,1) - deltaX_suave, posicoes_todas(p,2) - deltaY_suave, posicoes_todas(p,3)];
            
            T_alvo_se3 = se3(trvec2tform(alvo_global));
            T_local_se3 = inv(Tbase{p}) * T_alvo_se3;
            
            % Resolve a Cinemática Inversa com o novo alvo compensado
            [q_corrigido(p,:), ~] = iks{p}("link8", T_local_se3.tform, weights, ang_atuais(p,:));
            
        else % PERNA NO AR
            % Perna no ar não compensa o corpo, apenas segue o ciclo original
            q_corrigido(p, :) = ang_atuais(p, :);
        end
    end
    
    % --- 5. SAÍDA DE DADOS ---
    q1_corr = q_corrigido(1,:); 
    q2_corr = q_corrigido(2,:);
    q3_corr = q_corrigido(3,:); 
    q4_corr = q_corrigido(4,:);
    
    q1_novo = q1_corr; 
    q2_novo = q2_corr;
    q3_novo = q3_corr; 
    q4_novo = q4_corr;
end
%%
% function finalCoM = calculateCoM(leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_at, ...
%     ang1,ang2,ang3,ang4)
%     global Tbase
% 
%     matrizAng = [ang1;ang2;ang3;ang4];
%     matrizLegs = [leg_dir_af leg_esq_af leg_dir_at leg_esq_at];
%     tempx = 0;
%     tempy = 0;
%     aux = zeros(3,2);
%     CoM = zeros(4,2);
%     for k=1:4
% 
%         T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
%         "link8"));
%         aux1 = T.trvec;
% 
%         T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
%         "link6"));
%         aux2 = T.trvec;
%         tempx = (aux1(1,1) + aux2(1,1))/2;
%         tempy = (aux1(1,2) + aux2(1,2))/2;
%         aux(1,1) = tempx*1;
%         aux(1,2) = tempy*1;
% 
%         T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
%         "link4"));
%         aux3 = T.trvec;
% 
%         T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
%         "link3"));
%         aux4 = T.trvec;
% 
%         tempx = (aux3(1,1) + aux4(1,1))/2;
%         tempy = (aux3(1,2) + aux4(1,2))/2;
%         aux(2,1) = tempx*1;
%         aux(2,2) = tempy*1;
% 
%         T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
%         "link2"));
%         aux5 = T.trvec;
% 
%         T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
%         "link1"));
%         aux6 = T.trvec;
% 
%         tempx = (aux5(1,1) + aux6(1,1))/2;
%         tempy = (aux5(1,2) + aux6(1,2))/2;
%         aux(3,1) = tempx*1;
%         aux(3,2) = tempy*1;
%         CoM(k,:) = sum(aux);
%     end
%     xCom = (sum(CoM(:,1)))/14;
%     yCom = (sum(CoM(:,2)))/14;
%     finalCoM = [xCom,yCom];
% end
% 
% %%
% function [msm,P1,P2]  = calculaMSM(A,B,C,P)
% 
%     auxVetorLados = B-A;
%     auxVetorPonto = P-A;
%     d1 = (auxVetorLados(1)*auxVetorPonto(2) - auxVetorLados(2)*auxVetorPonto(1))/...
%     sqrt(((B(1)-A(1))^2)+(((B(2)-A(2))^2)));
%     auxVetorLados = C-B;
%     auxVetorPonto = P-B;
%     d2 = (auxVetorLados(1)*auxVetorPonto(2) - auxVetorLados(2)*auxVetorPonto(1))/...
%     sqrt(((C(1)-B(1))^2)+(((C(2)-B(2))^2)));
%     auxVetorLados = A-C;
%     auxVetorPonto = P-C;
%     d3 = (auxVetorLados(1)*auxVetorPonto(2) - auxVetorLados(2)*auxVetorPonto(1))/...
%     sqrt(((A(1)-C(1))^2)+(((A(2)-C(2))^2)));
%     [msm, idx] = min([d1, d2, d3]);
% 
%     % identificar lado crítico
%     if idx == 1
%         P1 = A;
%         P2 = B;
%     elseif idx == 2
%         P1 = B;
%         P2 = C;
%     else
%         P1 = C;
%         P2 = A;
%     end
% end
% %%
% function [lsm]  = calculaLSM(A,B,C,P)
%     syms t u
%     v = [1 0];
%     Rx = P(1) + t*v(1);
%     Ry = P(2) + t*v(2);
% 
%     %Lado AB
%     Sx = A(1) + u*(B(1)-A(1));
%     Sy = A(2) + u*(B(2)-A(2));
%     eq1 = Rx == Sx;
%     eq2 = Ry == Sy;
% 
%     solAB = solve([eq1,eq2],[t,u]);
%     tAB = double(solAB.t);
%     uAB = double(solAB.u);
% 
%     %Lado BC
%     Sx = B(1) + u*(C(1)-B(1));
%     Sy = B(2) + u*(C(2)-B(2));
%     eq1 = Rx == Sx;
%     eq2 = Ry == Sy;
% 
%     solBC = solve([eq1,eq2],[t,u]);
%     tBC = double(solBC.t);
%     uBC = double(solBC.u);
% 
%     %Lado CA
%     Sx = C(1) + u*(A(1)-C(1));
%     Sy = C(2) + u*(A(2)-C(2));
%     eq1 = Rx == Sx;
%     eq2 = Ry == Sy;
% 
%     solCA = solve([eq1,eq2],[t,u]);
%     tCA = double(solCA.t);
%     uCA = double(solCA.u);
% 
%     %Checar se os t são válidos e se a reta está dentro de u
%     t_validos = [];
% 
%     if ~isempty(tAB) && tAB>0 && uAB>=0 && uAB<=1
%         t_validos(end+1)=tAB;
%     end
%     if ~isempty(tBC) && tBC>0 && uBC>=0 && uBC<=1
%         t_validos(end+1)=tBC;
%     end
%     if ~isempty(tCA) && tCA>0 && uCA>=0 && uCA<=1
%         t_validos(end+1)=tCA;
%     end
%     if isempty(t_validos)
%         lsm = NaN;
%     else
%         lsm = min(t_validos);
%     end
% end
