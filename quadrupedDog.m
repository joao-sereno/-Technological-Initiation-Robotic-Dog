L1 = 36.75/100;
L2 = 45.75/100;
L3 = -50.5/100;
L4 = -153.5/100;
L5 = -130.5/100;

W = 130/100;
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
% r = rateControl(50);
% reset(r);
% Inicialização das variáveis antes do loop
% Inicialização das variáveis antes do loop
p = 0;
cycleSize = length(ikcy_dir_frente);
i = 1;
idx_hist = 1; % Índice independente para gravar o histórico contínuo

ik1 = inverseKinematics('RigidBodyTree', leg_dir_afrente);
ik2 = inverseKinematics('RigidBodyTree', leg_esq_afrente);
ik3 = inverseKinematics('RigidBodyTree', leg_dir_atras);
ik4 = inverseKinematics('RigidBodyTree', leg_esq_atras);
weights = [0 0 0 1 1 1];
ik2.SolverParameters.MaxIterations = 20;
ik2.SolverParameters.GradientTolerance = 1e-3;
ik2.SolverParameters.StepTolerance = 1e-3;

% Pré-alocando espaço com folga para as transições (multiplicado por 5 para garantir)
ang1_historico = zeros(cycleSize*5,3); 
ang2_historico = zeros(cycleSize*5,3);
ang3_historico = zeros(cycleSize*5,3);
ang4_historico = zeros(cycleSize*5,3);

ultimoAux = 1;
pos1 = [];
pos2 = [];
pos3 = [];
pos4 = [];

while(1)
    % A função retorna os ângulos que a perna deve assumir neste momento
    [ang1,ang2,ang3,ang4,ultimoAux,i,pos1,pos2,pos3,pos4] = jointAngles(cycleSize,offsetFirstLeg,...
    offsetSecondLeg,offsetThirdLeg,offsetFourthLeg,ikcy_dir_frente, ...
    ikcy_esq_frente,ikcy_dir_atras,ikcy_esq_atras,leg_dir_afrente, ...
    leg_esq_afrente,leg_dir_atras,leg_esq_atras,i,j,k,ik1,ik2,ik3,ik4, ...
    weights,p,ultimoAux,pos1,pos2,pos3,pos4);
    
    % --- INTERCEPTADOR DE TELETRANSPORTE ---
    % Se já temos histórico gravado, verificamos a diferença do último frame pro novo
    if idx_hist > 1
        % Calcula o tamanho do salto absoluto (em radianos) da Perna 1 e 2
        diff1 = max(abs(ang1(1,:) - ang1_historico(idx_hist-1,:)));
        diff2 = max(abs(ang2(1,:) - ang2_historico(idx_hist-1,:)));
        
        % Se o pulo for maior que ~0.05 radianos, é um teletransporte!
        if max(diff1, diff2) > 0.05
            N_frames = 20; % Quantidade de frames para suavizar a transição
            
            % Criando uma curva em "S" matemática nativa (Interpolação Cosseno)
            t = linspace(0, pi, N_frames)';
            s = (1 - cos(t)) / 2; % Vai de 0 a 1 suavemente
            
            % Aplica a curva para gerar a matriz de transição de N_frames x 3
            ang1 = (1-s) .* ang1_historico(idx_hist-1,:) + s .* ang1(1,:);
            ang2 = (1-s) .* ang2_historico(idx_hist-1,:) + s .* ang2(1,:);
            ang3 = (1-s) .* ang3_historico(idx_hist-1,:) + s .* ang3(1,:);
            ang4 = (1-s) .* ang4_historico(idx_hist-1,:) + s .* ang4(1,:);
        end
    end
    % ---------------------------------------
    
    % Pega a quantidade de pontos gerados (1 se normal, 20 se o interceptador rodou)
    num_passos = size(ang1, 1); 
    
    % Executa o buffer de pontos
    for step = 1:num_passos
        % Grava a linha correspondente no histórico
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

    p = p + 1;
    if(p==98)
        break;
    end
    
    i = i + 1
    if(i>cycleSize)
        break;
    end
end

% Cortando o excesso de zeros pré-alocados no final do script
ang1_historico = ang1_historico(1:idx_hist-1, :);
ang2_historico = ang2_historico(1:idx_hist-1, :);
ang3_historico = ang3_historico(1:idx_hist-1, :);
ang4_historico = ang4_historico(1:idx_hist-1, :);
%%
global d
d = rateControl(20);
total_frames = size(ang1_historico, 1); % NOVO: Pega o tamanho real com as transições
while(1)
    for i = 1:total_frames
        % Remover o if(i==200) break; se quiser ver a animação completa
        
        leg_dir_afrente.show(ang1_historico(i,:), FastUpdate=true,PreservePlot=false); hold on
        leg_esq_afrente.show(ang2_historico(i,:), FastUpdate=true,PreservePlot=false); 
        leg_dir_atras.show(ang3_historico(i,:), FastUpdate=true,PreservePlot=false);
        leg_esq_atras.show(ang4_historico(i,:), FastUpdate=true,PreservePlot=false);
        hold off
        d.waitfor;
    end
end
%%
global d
d = rateControl(200);
j = 1;
while(1)
     for i = 1:size(ikcy_dir_frente,1)
         leg_dir_afrente.show(gait(ikcy_dir_frente,i,offsetFirstLeg,false),FastUpdate=true,PreservePlot=false);hold on
         leg_esq_afrente.show(gait(ikcy_esq_frente,i,offsetSecondLeg,false),FastUpdate=true,PreservePlot=false);
         leg_dir_atras.show(gait(ikcy_dir_atras,i,offsetThirdLeg,false),FastUpdate=true, PreservePlot= false);
         leg_esq_atras.show(gait(ikcy_esq_atras,i,offsetFourthLeg,false),FastUpdate=true,PreservePlot=false);
         hold off
         d.waitfor;
     end
    j = j + 1;
    if (j > 2)
        break;
    end
end
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
    zu = -250/100; zd = -300/100;
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
function [ang1,ang2,ang3,ang4,ultimoAux,i,pos1_ultima,pos2_ultima,pos3_ultima, ...
    pos4_ultima] = jointAngles(cycleSize,o1,o2,o3,o4,ik_dir_af,...
    ik_esq_af,ik_dir_at,ik_esq_at,leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,i ...
    ,j,k,ik1,ik2,ik3,ik4,weights,p,ultimoAux,pos1_ultima,pos2_ultima,pos3_ultima, ...
    pos4_ultima)
        gait1 = mod(i + o1-1,cycleSize)+1;
        gait2 = mod(i + o2-1,cycleSize)+1;
        gait3 = mod(i + o3-1,cycleSize)+1;
        gait4 = mod(i + o4-1,cycleSize)+1;
        aux = 0;

        if(gait1>=k && gait1<=j)
            aux=1;
        elseif(gait2>=k && gait2<=j)
            aux=2;
        elseif(gait3>=k && gait3<=j)
            aux=3;
        elseif(gait4>=k && gait4<=j)
            aux=4;
        end
        if(ultimoAux==0 && aux==1)
            gait1 = mod(i + o1-1,cycleSize)+1;
            gait2 = mod(i + o2-1,cycleSize)+1;
            gait3 = mod(i + o3-1,cycleSize)+1;
            gait4 = mod(i + o4-1,cycleSize)+1;

            [~,q2_corrigido,q3_corrigido,~,ultimoAux,...
            pos1,~,~,pos4] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,ultimoAux);

            via1 = [pos1_ultima 
                    pos1];
            location1 = mstraj(via1,[],0.5,[],0.01,0.01);
            via4 = [pos4_ultima 
                    pos4];
            location4 = mstraj(via4,[],0.5,[],0.01,0.01);
            q1_corrigido = ikineTrajNum(leg_dir_af,se3(eye(3),location1),"link8",weights = [0 0 0 1 1 1]);
            q4_corrigido = ikineTrajNum(leg_esq_aft,se3(eye(3),location4),"link8",weights = [0 0 0 1 1 1]);
            length(q1_corrigido);
            length(q4_corrigido);
        elseif(ultimoAux==0 && aux==2)
            gait1 = mod(i + o1-1,cycleSize)+1;

            gait2 = mod(i + o2-1,cycleSize)+1;
            
            gait3 = mod(i + o3-1,cycleSize)+1;
            
            gait4 = mod(i + o4-1,cycleSize)+1;
            
            [~,q2_corrigido,q3_corrigido,q4_corrigido,ultimoAux,...
            pos1,~,~,~] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,ultimoAux);
            
            
            via1 = [pos1_ultima
            
            pos1];
            
            location1 = mstraj(via1,[],1,[],0.01,0.01);
            
            
            % Calcula a trajetória da perna 1
            
            q1_corrigido = ikineTrajNum(leg_dir_af,se3(eye(3),location1),"link8",weights = [0 0 0 1 1 1]);
            
            
            % --- ADICIONE ESTAS LINHAS ---
            
            % Descobre quantos passos a trajetória 1 gerou
            
            N = size(q1_corrigido, 1);
            
            
            % Repete os ângulos estáticos N vezes para igualar as dimensões
            
            q2_corrigido = repmat(q2_corrigido, N, 1);
            
            q3_corrigido = repmat(q3_corrigido, N, 1);
            
            q4_corrigido = repmat(q4_corrigido, N, 1); 


        elseif(ultimoAux==0 && aux==3)
            gait1 = mod(i + o1-1,cycleSize)+1;
            gait2 = mod(i + o2-1,cycleSize)+1;
            gait3 = mod(i + o3-1,cycleSize)+1;
            gait4 = mod(i + o4-1,cycleSize)+1;

            [q1_corrigido,~,~,q4_corrigido,ultimoAux,...
            ~,pos2,pos3,~] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,ultimoAux);

            via2 = [pos2_ultima 
                    pos2];
            location2 = mstraj(via2,[],0.5,[],0.01,0.01);
            via3 = [pos3_ultima 
                    pos3];
            location3 = mstraj(via3,[],0.5,[],0.01,0.01);
            q2_corrigido = ikineTrajNum(leg,se3(eye(3),location2),"link8",weights = [0 0 0 1 1 1]);
            q3_corrigido = ikineTrajNum(leg,se3(eye(3),location3),"link8",weights = [0 0 0 1 1 1]);
            length(q2_corrigido);
            length(q3_corrigido);
        elseif(ultimoAux==0 && aux==4)
            gait1 = mod(i + o1-1,cycleSize)+1;
            gait2 = mod(i + o2-1,cycleSize)+1;
            gait3 = mod(i + o3-1,cycleSize)+1;
            gait4 = mod(i + o4-1,cycleSize)+1;

            [q1_corrigido,q2_corrigido,~,~,ultimoAux,...
            ~,~,pos3,pos4] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,ultimoAux);

            via3 = [pos3_ultima 
                    pos3];
            location3 = mstraj(via3,[],0.5,[],0.01,0.01);
            via4 = [pos4_ultima 
                    pos4];
            location4 = mstraj(via4,[],0.5,[],0.01,0.01);
            q3_corrigido = ikineTrajNum(leg,se3(eye(3),location3),"link8",weights = [0 0 0 1 1 1]);
            q4_corrigido = ikineTrajNum(leg,se3(eye(3),location4),"link8",weights = [0 0 0 1 1 1]);
            length(q2_corrigido);
            length(q3_corrigido);
        else
            [q1_corrigido,q2_corrigido,q3_corrigido,q4_corrigido,ultimoAux,...
            ~,~,~,~] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,aux, ...
            ik1,ik2,ik3,ik4,weights,p,ultimoAux);
        end

        if(ultimoAux==1 && aux==0)
            gait1 = mod(i-1 + o1-1,cycleSize)+1;
            gait2 = mod(i-1 + o2-1,cycleSize)+1;
            gait3 = mod(i-1 + o3-1,cycleSize)+1;
            gait4 = mod(i-1 + o4-1,cycleSize)+1;
            [q1_corrigido,q2_corrigido,q3_corrigido,q4_corrigido,~,...
            pos1_ultima,pos2_ultima,~,~] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,aux);
            ultimoAux = 0;
            i = i + 1;
            pos1_ultima
        elseif(ultimoAux==2 && aux==0)
            gait1 = mod(i-1 + o1-1,cycleSize)+1;
            gait2 = mod(i-1 + o2-1,cycleSize)+1;
            gait3 = mod(i-1 + o3-1,cycleSize)+1;
            gait4 = mod(i-1 + o4-1,cycleSize)+1;
            [q1_corrigido,q2_corrigido,q3_corrigido,q4_corrigido,~,...
            ~,pos2_ultima,~,~] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,aux);
            ultimoAux = 0;
            i = i + 1;
        elseif(ultimoAux==3 && aux==0)
            gait1 = mod(i-1 + o1-1,cycleSize)+1;
            gait2 = mod(i-1 + o2-1,cycleSize)+1;
            gait3 = mod(i-1 + o3-1,cycleSize)+1;
            gait4 = mod(i-1 + o4-1,cycleSize)+1;
            [q1_corrigido,q2_corrigido,q3_corrigido,q4_corrigido,~,...
            ~,~,pos3_ultima,~] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,aux);
            ultimoAux = 0;
            i = i + 1;
        elseif(ultimoAux==4 && aux==0)
            gait1 = mod(i-1 + o1-1,cycleSize)+1;
            gait2 = mod(i-1 + o2-1,cycleSize)+1;
            gait3 = mod(i-1 + o3-1,cycleSize)+1;
            gait4 = mod(i-1 + o4-1,cycleSize)+1;
            [q1_corrigido,q2_corrigido,q3_corrigido,q4_corrigido,~,...
            ~,~,~,pos4_ultima] = ...
            fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
            ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,ultimoAux, ...
            ik1,ik2,ik3,ik4,weights,p,aux);
            ultimoAux = 0;
            i = i + 1;
        end

        ang1 = q1_corrigido;
        ang2 = q2_corrigido;
        ang3 = q3_corrigido;
        ang4 = q4_corrigido;

        % ang1 = saturate(round(rad2deg(q1_corrigido))+90,0,180);
        % ang2 = saturate(round(rad2deg(q2_corrigido))+90,0,180);
        % ang3 = saturate(round(rad2deg(q3_corrigido))+90,0,180);
        % ang4 = saturate(round(rad2deg(q4_corrigido))+90,0,180);
        % fprintf("\nang1_q1: %d, ang1_q2: %d , ang1_q3: %d\n" + ...
        % "ang2_q1: %d, ang2_q2: %d , ang2_q3: %d\n" + ...
        % "ang3_q1: %d, ang3_q2: %d , ang3_q3: %d\n" + ...
        % "ang4_q1: %d, ang4_q2: %d , ang4_q3: %d",ang1(1,1),ang1(1,2), ...
        %     ang1(1,3),ang2(1,1),ang2(1,2),ang2(1,3),ang3(1,1),ang3(1,2) ...
        %     ,ang3(1,3),ang4(1,1),ang4(1,2),ang4(1,3));
end
%%
function y = saturate(x,minVal,maxVal)
    y = min(max(x,minVal),maxVal);
end
%%
function [q1_corr,q2_corr,q3_corr,q4_corr,ultimoAux,posicao1,posicao2, ...
    posicao3, posicao4] = fowardKinematics(ang1,ang2,ang3,ang4,if_dir_af, ...
    if_esq_af,if_dir_at,if_esq_at,aux,ik1,ik2,ik3,ik4,weights,p,ultimoAux)
    global Tbase
    q1_corr = ang1;
    q2_corr = ang2;
    q3_corr = ang3;
    q4_corr = ang4;
    T = Tbase{1} * se3(if_dir_af.getTransform([ang1(1,1) ang1(1,2) ang1(1,3)],"link8"));
    aux1 = T.trvec;
    x1 = aux1(1,1);
    y1=aux1(1,2);
    T = Tbase{2} * se3(if_esq_af.getTransform([ang2(1,1) ang2(1,2) ang2(1,3)],"link8"));
    aux2 = T.trvec; 
    x2 = aux2(1,1);
    y2=aux2(1,2);
    T = Tbase{3} * se3(if_dir_at.getTransform([ang3(1,1) ang3(1,2) ang3(1,3)],"link8"));
    aux3 = T.trvec;
    x3 = aux3(1,1);
    y3=aux3(1,2);
    T = Tbase{4} * se3(if_esq_at.getTransform([ang4(1,1) ang4(1,2) ang4(1,3)],"link8"));
    aux4 = T.trvec;
    x4 = aux4(1,1);
    y4=aux4(1,2);
    
    CoM = calculateCoM(if_dir_af,if_esq_af,if_dir_at,if_esq_at,ang1,ang2 ...
        ,ang3,ang4);
    %disp(aux)
    posicao1 = [x1,y1,aux1(3)];
    posicao2 = [x2,y2,aux2(3)];
    posicao3 = [x3,y3,aux3(3)];
    posicao4 = [x4,y4,aux4(3)];
    if(aux==1)
        ultimoAux = aux;
            % poly = [x2 y2
            %     x3 y3
            %     x4 y4];
            % figure
            % hold on
            % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
            % axis equal
            % grid on
            % 
            % xq = CoM(1);
            % yq = CoM(2);
            % 
            % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
            % 
            % if in
            %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
            % else
            %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
            % end
            % hold off
        A = [x2 y2];
        B = [x3 y3];
        C = [x4 y4];
        %disp(CoM)
        area = ((B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1)));
        if area < 0
            temp = B;
            B = C;
            C = temp;
        end
        [MSM,P1,P2] = calculaMSM(A,B,C,CoM);
        MSM_min = 3.0/100;
        % if(MSM<MSM_min)
        %     fprintf("Robo instável: ");
        %     if(MSM<0)
        %         fprintf("Centro de massa fora do polígono")
        %     else
        %         fprintf("Centro de massa dentro do polígono, mas " + ...
        %             "ainda não chegou na posição do valor mínimo de margem")
        %     end
        % end
        %disp(MSM)
        LSM = calculaLSM(A,B,C,CoM);
        CoM_corrigido = CoM;
        if isnan(LSM) || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);
            
            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = MSM_desejado - MSM;
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % xq = CoM_corrigido(1);
            % yq = CoM_corrigido(2);
            % figure
            % hold on
            % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
            % axis equal
            % grid on
            % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
            % if in
            %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
            % else
            %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
            % end
        end

        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);
        
        altura2 = aux2(3);
        altura3 = aux3(3);
        altura4 = aux4(3);

        posicao2 = [x2-deltaX, y2-deltaY,altura2];
        posicao3 = [x3-deltaX, y3-deltaY,altura3];
        posicao4 = [x4-deltaX, y4-deltaY,altura4];

        T2 = trvec2tform(posicao2);
        T3 = trvec2tform(posicao3);
        T4 = trvec2tform(posicao4);

        [q2_corr, ~] = ik2("link8", T2, weights, q2_corr);
        [q3_corr, ~] = ik3("link8", T3, weights, q3_corr);
        [q4_corr, ~] = ik4("link8", T4, weights, q4_corr);
        %p
        if(p==95)
            pontos_x = [x1; x2-deltaX; x3-deltaX; x4-deltaX];
            pontos_y = [y1; y2-deltaY; y3-deltaY; y4-deltaY];
            [K, ~] = convhull(pontos_x, pontos_y);
            poly = [pontos_x(K), pontos_y(K)];
        
            figure
            hold on
            plot(poly(:,1), poly(:,2), 'k-', 'LineWidth', 2) % Desenha o perímetro correto
            plot(pontos_x, pontos_y, 'ko', 'MarkerFaceColor', 'k') % Desenha os 4 apoios
            axis equal
            grid on
        
            xq = CoM_corrigido(1);
            yq = CoM_corrigido(2);
        
            [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
        
            if in
                plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
            else
                plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
            end
            hold off
        end

    elseif(aux==2)
        ultimoAux = aux;
        % 
        % poly = [x1 y1
        %         x3 y3
        %         x4 y4];
        % 
        % figure
        % hold on
        % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
        % axis equal
        % grid on
        % 
        % xq = CoM(1);
        % yq = CoM(2);
        % 
        % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
        % 
        % if in
        %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
        % else
        %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
        % end
        % hold off
        A = [x1 y1];
        B = [x3 y3];
        C = [x4 y4];
        %disp(CoM);
        calculaMSM(A,B,C,CoM);
        area = ((B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1)));
        if area < 0
            temp = B;
            B = C;
            C = temp;
        end
        [MSM,P1,P2] = calculaMSM(A,B,C,CoM);
        MSM_min = 3.0/100;
        % if(MSM<MSM_min)
        %     fprintf("Robo instável: ");
        %     if(MSM<0)
        %         fprintf("Centro de massa fora do polígono")
        %     else
        %         fprintf("Centro de massa dentro do polígono, mas " + ...
        %             "ainda não chegou na posição do valor mínimo de margem")
        %     end
        % end
        %disp(MSM)
        LSM = calculaLSM(A,B,C,CoM);
        CoM_corrigido = CoM;
        if isnan(LSM) || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);
            
            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = MSM_desejado - MSM;
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % xq = CoM_corrigido(1);
            % yq = CoM_corrigido(2);
            % figure
            % hold on
            % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
            % axis equal
            % grid on
            % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
            % if in
            %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
            % else
            %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
            % end
        end

        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);
        
        altura1 = aux1(3);
        altura3 = aux3(3);
        altura4 = aux4(3);

        posicao1 = [x1-deltaX, y1-deltaY,altura1];
        posicao3 = [x3-deltaX, y3-deltaY,altura3];
        posicao4 = [x4-deltaX, y4-deltaY,altura4];

        T1 = trvec2tform(posicao1);
        T3 = trvec2tform(posicao3);
        T4 = trvec2tform(posicao4);

        [q1_corr, ~] = ik1("link8", T1, weights, q1_corr);
        [q3_corr, ~] = ik3("link8", T3, weights, q3_corr);
        [q4_corr, ~] = ik4("link8", T4, weights, q4_corr);

    elseif(aux==3)
        ultimoAux = aux;
        % poly = [x1 y1
        %         x2 y2
        %         x4 y4];
        % 
        % figure
        % hold on
        % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
        % axis equal
        % grid on
        % 
        % xq = CoM(1);
        % yq = CoM(2);
        % 
        % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
        % 
        % if in
        %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
        % else
        %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
        % end
        % hold off
        % disp(CoM)
        A = [x1 y1];
        B = [x2 y2];
        C = [x4 y4];
        calculaMSM(A,B,C,CoM);
        area = ((B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1)));
        if area < 0
            temp = B;
            B = C;
            C = temp;
        end
        [MSM,P1,P2] = calculaMSM(A,B,C,CoM);
        MSM_min = 3.0/100;
        % if(MSM<MSM_min)
        %     fprintf("Robo instável: ");
        %     if(MSM<0)
        %         fprintf("Centro de massa fora do polígono")
        %     else
        %         fprintf("Centro de massa dentro do polígono, mas " + ...
        %             "ainda não chegou na posição do valor mínimo de margem")
        %     end
        % end
        %disp(MSM)
        LSM = calculaLSM(A,B,C,CoM);
        CoM_corrigido = CoM;
        if isnan(LSM) || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);
            
            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = MSM_desejado - MSM;
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % xq = CoM_corrigido(1);
            % yq = CoM_corrigido(2);
            % figure
            % hold on
            % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
            % axis equal
            % grid on
            % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
            % if in
            %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
            % else
            %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
            % end
        end

        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);
        
        altura1 = aux1(3);
        altura2 = aux2(3);
        altura4 = aux4(3);

        posicao1 = [x1-deltaX, y1-deltaY,altura1];
        posicao2 = [x2-deltaX, y2-deltaY,altura2];
        posicao4 = [x4-deltaX, y4-deltaY,altura4];

        T1 = trvec2tform(posicao1);
        T2 = trvec2tform(posicao2);
        T4 = trvec2tform(posicao4);

        [q1_corr, ~] = ik1("link8", T1, weights, q1_corr);
        [q2_corr, ~] = ik2("link8", T2, weights, q2_corr);
        [q4_corr, ~] = ik4("link8", T4, weights, q4_corr);

    elseif(aux==4)
        ultimoAux = aux;
        % poly = [x1 y1
        %         x2 y2
        %         x3 y3];
        % figure
        % hold on
        % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
        % axis equal
        % grid on
        % 
        % xq = CoM(1);
        % yq = CoM(2);
        % 
        % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
        % 
        % if in
        %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
        % else
        %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
        % end
        % hold off
        % disp(CoM)
        A = [x1 y1];
        B = [x2 y2];
        C = [x3 y3];
        %disp(CoM);
        calculaMSM(A,B,C,CoM);
        area = ((B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1)));
        if area < 0
            temp = B;
            B = C;
            C = temp;
        end
        [MSM,P1,P2] = calculaMSM(A,B,C,CoM);
        MSM_min = 3.0/100;
        % if(MSM<MSM_min)
        %     fprintf("Robo instável: ");
        %     if(MSM<0)
        %         fprintf("Centro de massa fora do polígono")
        %     else
        %         fprintf("Centro de massa dentro do polígono, mas " + ...
        %             "ainda não chegou na posição do valor mínimo de margem")
        %     end
        % end
        %disp(MSM)
        LSM = calculaLSM(A,B,C,CoM);
        CoM_corrigido = CoM;
        if isnan(LSM) || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);
            
            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = MSM_desejado - MSM;
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % xq = CoM_corrigido(1);
            % yq = CoM_corrigido(2);
            % figure
            % hold on
            % plot([poly(:,1); poly(1,1)], [poly(:,2); poly(1,2)], 'k-')
            % axis equal
            % grid on
            % [in,on] = inpolygon(xq,yq,poly(:,1),poly(:,2));
            % if in
            %     plot(xq,yq,'r+','MarkerSize',10,'LineWidth',2)
            % else
            %     plot(xq,yq,'bo','MarkerSize',10,'LineWidth',2)
            % end
        end

        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);
        
        altura1 = aux1(3);
        altura2 = aux2(3);
        altura3 = aux3(3);

        posicao1 = [x1-deltaX, y1-deltaY,altura1];
        posicao2 = [x2-deltaX, y2-deltaY,altura2];
        posicao3 = [x3-deltaX, y3-deltaY,altura3];

        T1 = trvec2tform(posicao1);
        T2 = trvec2tform(posicao2);
        T3 = trvec2tform(posicao3);

        [q1_corr, ~] = ik1("link8", T1, weights, q1_corr);
        [q2_corr, ~] = ik2("link8", T2, weights, q2_corr);
        [q3_corr, ~] = ik3("link8", T3, weights, q3_corr);

    end

end
%%
function finalCoM = calculateCoM(leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_at, ...
    ang1,ang2,ang3,ang4)
    global Tbase

    matrizAng = [ang1;ang2;ang3;ang4];
    matrizLegs = [leg_dir_af leg_esq_af leg_dir_at leg_esq_at];
    tempx = 0;
    tempy = 0;
    aux = zeros(3,2);
    CoM = zeros(4,2);
    for k=1:4

        T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
        "link8"));
        aux1 = T.trvec;

        T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
        "link6"));
        aux2 = T.trvec;
        tempx = (aux1(1,1) + aux2(1,1))/2;
        tempy = (aux1(1,2) + aux2(1,2))/2;
        aux(1,1) = tempx*1;
        aux(1,2) = tempy*1;

        T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
        "link4"));
        aux3 = T.trvec;

        T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
        "link3"));
        aux4 = T.trvec;

        tempx = (aux3(1,1) + aux4(1,1))/2;
        tempy = (aux3(1,2) + aux4(1,2))/2;
        aux(2,1) = tempx*1;
        aux(2,2) = tempy*1;

        T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
        "link2"));
        aux5 = T.trvec;

        T = Tbase{k} * se3(matrizLegs(k).getTransform([matrizAng(k,1) matrizAng(k,2) matrizAng(k,3)], ...
        "link1"));
        aux6 = T.trvec;

        tempx = (aux5(1,1) + aux6(1,1))/2;
        tempy = (aux5(1,2) + aux6(1,2))/2;
        aux(3,1) = tempx*1;
        aux(3,2) = tempy*1;
        CoM(k,:) = sum(aux);
    end
    xCom = (sum(CoM(:,1)))/14;
    yCom = (sum(CoM(:,2)))/14;
    finalCoM = [xCom,yCom];
end

%%
function [msm,P1,P2]  = calculaMSM(A,B,C,P)
    auxVetorLados = B-A;
    auxVetorPonto = P-A;
    d1 = (auxVetorLados(1)*auxVetorPonto(2) - auxVetorLados(2)*auxVetorPonto(1))/...
    sqrt(((B(1)-A(1))^2)+(((B(2)-A(2))^2)));
    auxVetorLados = C-B;
    auxVetorPonto = P-B;
    d2 = (auxVetorLados(1)*auxVetorPonto(2) - auxVetorLados(2)*auxVetorPonto(1))/...
    sqrt(((C(1)-B(1))^2)+(((C(2)-B(2))^2)));
    auxVetorLados = A-C;
    auxVetorPonto = P-C;
    d3 = (auxVetorLados(1)*auxVetorPonto(2) - auxVetorLados(2)*auxVetorPonto(1))/...
    sqrt(((A(1)-C(1))^2)+(((A(2)-C(2))^2)));
    [msm, idx] = min([d1, d2, d3]);

    % identificar lado crítico
    if idx == 1
        P1 = A;
        P2 = B;
    elseif idx == 2
        P1 = B;
        P2 = C;
    else
        P1 = C;
        P2 = A;
    end
end
%%
function [lsm]  = calculaLSM(A,B,C,P)
    syms t u
    v = [1 0];
    Rx = P(1) + t*v(1);
    Ry = P(2) + t*v(2);
    %Lado AB
    Sx = A(1) + u*(B(1)-A(1));
    Sy = A(2) + u*(B(2)-A(2));
    eq1 = Rx == Sx;
    eq2 = Ry == Sy;

    solAB = solve([eq1,eq2],[t,u]);
    tAB = double(solAB.t);
    uAB = double(solAB.u);

    %Lado BC
    Sx = B(1) + u*(C(1)-B(1));
    Sy = B(2) + u*(C(2)-B(2));
    eq1 = Rx == Sx;
    eq2 = Ry == Sy;

    solBC = solve([eq1,eq2],[t,u]);
    tBC = double(solBC.t);
    uBC = double(solBC.u);

    %Lado CA
    Sx = C(1) + u*(A(1)-C(1));
    Sy = C(2) + u*(A(2)-C(2));
    eq1 = Rx == Sx;
    eq2 = Ry == Sy;

    solCA = solve([eq1,eq2],[t,u]);
    tCA = double(solCA.t);
    uCA = double(solCA.u);
    
    % fprintf("tAB: %f uAB: %f tBC: %f uBC: %f tCB: %f uCB: %f",tAB,uAB,tBC,uBC ...
    %     ,tCA,uCA);
    %Checar se os t são válidos e se a reta está dentro de u
    t_validos = [];

    if ~isempty(tAB) && tAB>0 && uAB>=0 && uAB<=1
        t_validos(end+1)=tAB;
    end
    if ~isempty(tBC) && tBC>0 && uBC>=0 && uBC<=1
        t_validos(end+1)=tBC;
    end
    if ~isempty(tCA) && tCA>0 && uCA>=0 && uCA<=1
        t_validos(end+1)=tCA;
    end
    if isempty(t_validos)
        lsm = NaN;
    else
        lsm = min(t_validos);
    end
end
