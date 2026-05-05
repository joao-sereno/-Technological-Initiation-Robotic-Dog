L1 = 36.75/100;
L2 = 45.75/100;
L3 = -50.5/100;
L4 = -153.5/100;
L5 = -130.5/100;

W = 180/100;
L = 300/100;

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
r = rateControl(200);
reset(r);
cycleSize = length(ikcy_dir_frente);
h = 1;
i = 1;
global q1_corr q2_corr q3_corr q4_corr inicializado

q1_corr = [];
q2_corr = [];
q3_corr = [];
q4_corr = [];
inicializado = [];
global aux
aux = 0;
ang1_hist = zeros(cycleSize,3);
ang2_hist = zeros(cycleSize,3);
ang3_hist = zeros(cycleSize,3);
ang4_hist = zeros(cycleSize,3);
while(1)
    [ang1,ang2,ang3,ang4,aux] = jointAngles(cycleSize,offsetFirstLeg,...
    offsetSecondLeg,offsetThirdLeg,offsetFourthLeg,ikcy_dir_frente, ...
    ikcy_esq_frente,ikcy_dir_atras,ikcy_esq_atras,leg_dir_afrente, ...
    leg_esq_afrente,leg_dir_atras,leg_esq_atras,i,j,k,aux);
    angles = sprintf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d"...
    ,ang1(1,1),ang1(1,2),ang1(1,3),ang2(1,1),...
    ang2(1,2),ang2(1,3),ang3(1,1),ang3(1,2),ang3(1,3),ang4(1,1),ang4...
    (1,2),ang4(1,3));

    ang1_hist(i,:) = ang1;
    ang2_hist(i,:) = ang2;
    ang3_hist(i,:) = ang3;
    ang4_hist(i,:) = ang4;
    %writeline(device,angles);
    i = i + 1;
    if(i>cycleSize)
        %i = 1;
        break;
    end
      % h = h + 1;
      % if (h > 1)
      %       break;
      %  end
    r.waitfor;
end
%%
global d
d = rateControl(50);

for i = 1:cycleSize

    leg_dir_afrente.show(ang1_hist(i,:), ...
        FastUpdate=true,PreservePlot=false); hold on

    leg_esq_afrente.show(ang2_hist(i,:), ...
        FastUpdate=true,PreservePlot=false);

    leg_dir_atras.show(ang3_hist(i,:), ...
        FastUpdate=true,PreservePlot=false);

    leg_esq_atras.show(ang4_hist(i,:), ...
        FastUpdate=true,PreservePlot=false);

    hold off
    d.waitfor;

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
function [ang1,ang2,ang3,ang4,aux] = jointAngles(cycleSize,o1,o2,o3,o4,ik_dir_af,...
    ik_esq_af,ik_dir_at,ik_esq_at,leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,i ...
    ,j,k,aux)
        gait1 = mod(i + o1-1,cycleSize)+1;
        gait2 = mod(i + o2-1,cycleSize)+1;
        gait3 = mod(i + o3-1,cycleSize)+1;
        gait4 = mod(i + o4-1,cycleSize)+1;
        % fprintf("gait 1: %d\n gait 2: %d\n gait 3: %d\n gait 4: %d\n", ...
        %     gait1,gait2,gait3,gait4);
        if(gait1>=k && gait1<=j)
            aux=1;
        elseif(gait2>=k && gait2<=j)
            aux=2;
        elseif(gait3>=k && gait3<=j)
            aux=3;
        elseif(gait4>=k && gait4<=j)
            aux=4;
        end
        % ang1 = 1;
        % ang2 = ang1;
        % ang3 = ang2;
        % ang4 = ang3;
        % fprintf("aux: %d\n",aux);
        [q1_corrigido,q2_corrigido,q3_corrigido,q4_corrigido] = ...
        fowardKinematics(ik_dir_af(gait1,:),ik_esq_af(gait2,:),ik_dir_at(gait3,:) ...
        ,ik_esq_at(gait4,:),leg_dir_af,leg_esq_af,leg_dir_at,leg_esq_aft,aux);

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
function [q1_novo,q2_novo,q3_novo,q4_novo] = fowardKinematics(ang1,ang2,ang3,ang4,if_dir_af,if_esq_af, ...
    if_dir_at,if_esq_at,aux)
    
    global q1_corr q2_corr q3_corr q4_corr inicializado

    if isempty(inicializado)
        q1_corr = ang1;
        q2_corr = ang2;
        q3_corr = ang3;
        q4_corr = ang4;
        inicializado = true;
    end
    
    global Tbase
    
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
    disp(aux)
    if(aux==1)
        q1 = q1_corr;
        q2 = q2_corr;
        q3 = q3_corr;
        q4 = q4_corr;
        poly = [x2 y2
                x3 y3
                x4 y4];
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
        % xlabel('Eixo x')
        % ylabel('Eixo y')
        % title('Gráfico polígono de suporte com o Centro de Massa')
        A = [x2 y2];
        B = [x3 y3];
        C = [x4 y4];
        % hold off
        % disp(CoM)
        area = (B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1));
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
        if isnan(LSM) || LSM < MSM_min || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);

            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = 5*(MSM_desejado - MSM);
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % disp(CoM_corrigido);
            xq = CoM_corrigido(1);
            yq = CoM_corrigido(2);
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
            % xlabel('Eixo x')
            % ylabel('Eixo y')
            % title('Gráfico polígono de suporte com o Centro de Massa')
        end
        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);

        %Jacobiano que resolverá o deslocamento necessário para
        %estabilizar o robo.
        delta_vec = [-deltaX; -deltaY; 0];
    
        % PERNA 2
        J2 = geometricJacobian(if_esq_af, q2, "link8");
        dq2 = pinv(J2(1:3,:)) * delta_vec;
        q2_corr = q2_corr + dq2';
    
        % PERNA 3
        J3 = geometricJacobian(if_dir_at, q3, "link8");
        dq3 = pinv(J3(1:3,:)) * delta_vec;
        q3_corr = q3_corr + dq3';
    
        % PERNA 4
        J4 = geometricJacobian(if_esq_at, q4, "link8");
        dq4 = pinv(J4(1:3,:)) * delta_vec;
        q4_corr = q4_corr + dq4';
        % z2 = aux2(1,3);
        % z3 = aux3(1,3);
        % z4 = aux4(1,3);
        % 
        % pos2 = [x2 - deltaX, y2 - deltaY, z2];
        % pos3 = [x3 - deltaX, y3 - deltaY, z3];
        % pos4 = [x4 - deltaX, y4 - deltaY, z4];

        % q2_novo = ikineTrajNum(if_esq_af, se3(eye(3),pos2), "link8", weights=[0 0 0 1 1 1]);
        % q3_novo = ikineTrajNum(if_dir_at, se3(eye(3),pos3), "link8", weights=[0 0 0 1 1 1]);
        % q4_novo = ikineTrajNum(if_esq_at, se3(eye(3),pos4), "link8", weights=[0 0 0 1 1 1]);
        % 
        % q2_novo = q2_novo(end,:);
        % q3_novo = q3_novo(end,:);
        % q4_novo = q4_novo(end,:);
        % T2 = se3(eye(3), pos2);
        % T3 = se3(eye(3), pos3);
        % T4 = se3(eye(3), pos4);
        % 
        % q2_novo = ikine(if_esq_af, T2, "link8", ang2, "Weights", [0 0 0 1 1 1]);
        % q3_novo = ikine(if_dir_at, T3, "link8", ang3, "Weights", [0 0 0 1 1 1]);
        % q4_novo = ikine(if_esq_at, T4, "link8", ang4, "Weights", [0 0 0 1 1 1]);
        % 
        % ang2 = q2_novo;
        % ang3 = q3_novo;
        % ang4 = q4_novo;
    elseif(aux==2)
        q1 = q1_corr;
        q2 = q2_corr;
        q3 = q3_corr;
        q4 = q4_corr;
        poly = [x1 y1
                x3 y3
                x4 y4];

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
        A = [x1 y1];
        B = [x3 y3];
        C = [x4 y4];
        % hold off
        % disp(CoM)
        area = (B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1));
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
        if isnan(LSM) || LSM < MSM_min || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);

            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = 5*(MSM_desejado - MSM);
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % disp(CoM_corrigido);
            xq = CoM_corrigido(1);
            yq = CoM_corrigido(2);
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
            % xlabel('Eixo x')
            % ylabel('Eixo y')
            % title('Gráfico polígono de suporte com o Centro de Massa')
        end
        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);
        delta_vec = [-deltaX; -deltaY; 0];
        % PERNA 1
        J1 = geometricJacobian(if_dir_af, q1, "link8");
        dq1 = pinv(J1(1:3,:)) * delta_vec;
        q1_corr = q1_corr + dq1';
    
        % PERNA 3
        J3 = geometricJacobian(if_dir_at, q3, "link8");
        dq3 = pinv(J3(1:3,:)) * delta_vec;
        q3_corr = q3_corr + dq3';
    
        % PERNA 4
        J4 = geometricJacobian(if_esq_at, q4, "link8");
        dq4 = pinv(J4(1:3,:)) * delta_vec;
        q4_corr = q4_corr + dq4';
        % z1 = aux1(1,3);
        % z3 = aux3(1,3);
        % z4 = aux4(1,3);
        % 
        % pos1 = [x1 - deltaX, y1 - deltaY, z1];
        % pos3 = [x3 - deltaX, y3 - deltaY, z3];
        % pos4 = [x4 - deltaX, y4 - deltaY, z4];

        % q1_novo = ikineTrajNum(if_esq_af, se3(eye(3),pos1), "link8", weights=[0 0 0 1 1 1]);
        % q3_novo = ikineTrajNum(if_dir_at, se3(eye(3),pos3), "link8", weights=[0 0 0 1 1 1]);
        % q4_novo = ikineTrajNum(if_esq_at, se3(eye(3),pos4), "link8", weights=[0 0 0 1 1 1]);
        % 
        % q1_novo = q1_novo(end,:);
        % q3_novo = q3_novo(end,:);
        % q4_novo = q4_novo(end,:);
        % 
        % T1 = se3(eye(3), pos1);
        % T3 = se3(eye(3), pos3);
        % T4 = se3(eye(3), pos4);
        % 
        % q1_novo = ikine(if_esq_af, T1, "link8", ang1, "Weights", [0 0 0 1 1 1]);
        % q3_novo = ikine(if_dir_at, T3, "link8", ang3, "Weights", [0 0 0 1 1 1]);
        % q4_novo = ikine(if_esq_at, T4, "link8", ang4, "Weights", [0 0 0 1 1 1]);
        % 
        % ang1 = q1_novo;
        % ang3 = q3_novo;
        % ang4 = q4_novo;
        
    elseif(aux==3)
        q1 = q1_corr;
        q2 = q2_corr;
        q3 = q3_corr;
        q4 = q4_corr;
        poly = [x1 y1
                x2 y2
                x4 y4];
        
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
        B = [x2 y2];
        C = [x4 y4];
        %disp(CoM);
         area = (B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1));
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
        if isnan(LSM) || LSM < MSM_min || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);

            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = 5*(MSM_desejado - MSM);
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % disp(CoM_corrigido);
            xq = CoM_corrigido(1);
            yq = CoM_corrigido(2);
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
            % xlabel('Eixo x')
            % ylabel('Eixo y')
            % title('Gráfico polígono de suporte com o Centro de Massa')
        end
        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);
        delta_vec = [-deltaX; -deltaY; 0];
        % PERNA 1
        J1 = geometricJacobian(if_dir_af, q1, "link8");
        dq1 = pinv(J1(1:3,:)) * delta_vec;
        q1_corr = q1_corr + dq1';
    
        % PERNA 2
        J2 = geometricJacobian(if_esq_af, q2, "link8");
        dq2 = pinv(J2(1:3,:)) * delta_vec;
        q2_corr = q2_corr + dq2';
    
        % PERNA 4
        J4 = geometricJacobian(if_esq_at, q4, "link8");
        dq4 = pinv(J4(1:3,:)) * delta_vec;
        q4_corr = q4_corr + dq4';
        % z1 = aux1(1,3);
        % z2 = aux2(1,3);
        % z4 = aux4(1,3);
        % 
        % pos1 = [x1 - deltaX, y1 - deltaY, z1];
        % pos2 = [x2 - deltaX, y2 - deltaY, z2];
        % pos4 = [x4 - deltaX, y4 - deltaY, z4];
        % 
        % T1 = se3(eye(3), pos1);
        % T2 = se3(eye(3), pos2);
        % T4 = se3(eye(3), pos4);
        % 
        % q1_novo = ikine(if_esq_af, T1, "link8", ang1, "Weights", [0 0 0 1 1 1]);
        % q2_novo = ikine(if_dir_at, T2, "link8", ang2, "Weights", [0 0 0 1 1 1]);
        % q4_novo = ikine(if_esq_at, T4, "link8", ang4, "Weights", [0 0 0 1 1 1]);
        % 
        % ang1 = q1_novo;
        % ang2 = q2_novo;
        % ang4 = q4_novo;
    elseif(aux==4)
        q1 = q1_corr;
        q2 = q2_corr;
        q3 = q3_corr;
        q4 = q4_corr;
        poly = [x1 y1
                x2 y2
                x3 y3];
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
        B = [x2 y2];
        C = [x3 y3];
        % disp(CoM)
         area = (B(1)-A(1))*(C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1));
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
        if isnan(LSM) || LSM < MSM || LSM < MSM
            e = P2 - P1;
            n = [-e(2),e(1)];
            n_unitario= n/norm(n);

            centro = (A+B+C)/3;
            if dot(n_unitario,centro-CoM)<0
                n_unitario = -n_unitario;
            end
            MSM_desejado = 0.03;
            delta = 5*(MSM_desejado - MSM);
            deltaP = delta*n_unitario;
            CoM_corrigido = CoM + deltaP;
            % disp(CoM_corrigido);
            xq = CoM_corrigido(1);
            yq = CoM_corrigido(2);
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
            % xlabel('Eixo x')
            % ylabel('Eixo y')
            % title('Gráfico polígono de suporte com o Centro de Massa')
        end
        deltaX = CoM_corrigido(1) - CoM(1);
        deltaY = CoM_corrigido(2) - CoM(2);
        delta_vec = [-deltaX; -deltaY; 0];
        % PERNA 1
        J1 = geometricJacobian(if_dir_af, q1, "link8");
        dq1 = pinv(J1(1:3,:)) * delta_vec;
        q1_corr = q1_corr + dq1';
    
        % PERNA 2
        J2 = geometricJacobian(if_esq_af, q2, "link8");
        dq2 = pinv(J2(1:3,:)) * delta_vec;
        q2_corr = q2_corr + dq2';
    
        % PERNA 4
        J3 = geometricJacobian(if_dir_at, q3, "link8");
        dq3 = pinv(J3(1:3,:)) * delta_vec;
        q3_corr = q3_corr + dq3';
        % z1 = aux1(1,3);
        % z2 = aux2(1,3);
        % z3 = aux3(1,3);
        % 
        % pos1 = [x1 - deltaX, y1 - deltaY, z1];
        % pos2 = [x2 - deltaX, y2 - deltaY, z2];
        % pos3 = [x3 - deltaX, y3 - deltaY, z3];

        % q1_novo = ikineTrajNum(if_esq_af, se3(eye(3),pos1), "link8", weights=[0 0 0 1 1 1]);
        % q2_novo = ikineTrajNum(if_dir_at, se3(eye(3),pos2), "link8", weights=[0 0 0 1 1 1]);
        % q3_novo = ikineTrajNum(if_esq_at, se3(eye(3),pos3), "link8", weights=[0 0 0 1 1 1]);
        % 
        % q1_novo = q1_novo(end,:);
        % q2_novo = q2_novo(end,:);
        % q3_novo = q3_novo(end,:);
        % T1 = se3(eye(3), pos1);
        % T2 = se3(eye(3), pos2);
        % T3 = se3(eye(3), pos3);
        % 
        % q1_novo = ikine(if_esq_af, T1, "link8", ang1, "Weights", [0 0 0 1 1 1]);
        % q2_novo = ikine(if_dir_at, T2, "link8", ang2, "Weights", [0 0 0 1 1 1]);
        % q3_novo = ikine(if_esq_at, T3, "link8", ang3, "Weights", [0 0 0 1 1 1]);
        % 
        % ang1 = q1_novo;
        % ang2 = q2_novo;
        % ang3 = q3_novo;

    end
    q1_novo = q1_corr;
    q2_novo = q2_corr;
    q3_novo = q3_corr;
    q4_novo = q4_corr;
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
