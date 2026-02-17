%Selecionando portas COM livres para começar a comunicação serial entre
%Matlab e o Arduino. 
portasDisponiveis = serialportlist;
device = serialport(portasDisponiveis(1),9600);
configureTerminator(device,"LF")
%%
%Definindo as matrizes de transformações homogêneas

L1 = 20.75/100;
L2 = 33.75/100;
L3 = -28.5/100;
L4 = -88.4/100;
L5 = -68.4/100;

[e1_afrente,e1_atras,e2_afrente,e2_atras] = creatHomoMatrix(L1,L2,L3,L4,L5);
%%
%Transformando as matrizes em corpos rigídos com formato/estrutura de
%arvore(Rigid Body Tree Structer).
[leg1_afrente,leg1_atras,leg2_afrente,leg2_atras] = transformHomoMatrixInTree(e1_afrente,e1_atras,e2_afrente,e2_atras);
%%
%Mostrando como estão as pernas no espaço 3D, quando todos os angulos das 
%juntas q1,q2 e q3 em radianos estão em 0. Conhecido como zero
%configuration
showZeroConfig(leg1_afrente,leg1_atras,leg2_afrente,leg2_atras);
%%
%qcycle1 e qcycle2 (Andar para frente com a perna direita e esquerda)
[ikineCycle1Afrente,ikineCycle1Atras,ikineCycle2Afrente,ikineCycle2Atras,k] =... 
    qcycleByVia(L2,-L2,L2,-L2,-L1,L1,L1,-L1,leg1_afrente,leg1_atras,leg2_afrente,...
    leg2_atras);
%%
%Definindo os offsets com base no valor k, que determina o momento de
%subida no plano zx ou zy da trajetória.
auxSize = length(ikineCycle1Afrente);
[offsetFirstLeg,offsetSecondLeg,offsetThirdLeg,offsetFourthLeg] = offSet(k,auxSize);
%%
r = rateControl(50);
reset(r);
cycleSize = length(ikineCycle1Afrente);
j = 1;
i = 1;
while(1)
    [ang1,ang2,ang3,ang4] = jointAngles(cycleSize,offsetFirstLeg,...
    offsetSecondLeg,offSetThirdLeg,offSetFourthLeg,ikineCycle1Afrente, ...
    ikineCycle1Atras,ikineCycle2Afrente,ikineCycle2Atras,i);
    angles = sprintf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n"...
    ,ang1(1,1),ang1(1,2),ang1(1,3),ang2(1,1),...
    ang2(1,2),ang2(1,3),ang3(1,1),ang3(1,2),ang3(1,3),ang4(1,1),ang4...
    (1,2),ang4(1,3));
    writeline(device,angles);
    i = i + 1;
    if(i>cycleSize)
        i = 1;
    end
    j = j + 1;
    if (j > 1)
        break;
    end
end
%%
global d
d = rateControl(200);
j = 1;
while(1)
     for i = 1:size(ikineCycle1Afrente,1)
         leg1_afrente.show(gait(ikineCycle1Afrente,i,offsetFirstLeg,false),FastUpdate=true,PreservePlot=false);hold on
         leg2_afrente.show(gait(ikineCycle2Afrente,i,offsetSecondLeg,false),FastUpdate=true,PreservePlot=false);
         leg1_atras.show(gait(ikineCycle1Atras,i,offsetThirdLeg,false),FastUpdate=true, PreservePlot= false);
         leg2_atras.show(gait(ikineCycle2Atras,i,offsetFourthLeg,false),FastUpdate=true,PreservePlot=false);
         hold off
         d.waitfor;
     end
    j = j + 1;
    if (j > 3)
        break;
    end
end
%% 
function [e1,e2,e3,e4] = creatHomoMatrix(L1,L2,L3,L4,L5)
    e1 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    e2 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    e3 = ETS3.Rx("q1")*ETS3.Ty(L1)*ETS3.Tx(L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
    e4 = ETS3.Rx("q1")*ETS3.Ty(-L1)*ETS3.Tx(-L2)*ETS3.Tz(L3)*ETS3.Ry("q2")*ETS3.Tz(L4)*ETS3.Ry("q3")*ETS3.Tz(L5);
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
function [qcycle1Afrente,qcycle1Atras,qcycle2Afrente,qcycle2Atras,k] = ...
    qcycleByVia(x1,x2,x3,x4,y1,y2,y3,y4,leg1Af,leg1At,leg2Af,leg2At)
    zu = -120/100; zd = -140/100;
    xf1 = 40/100+x1; xb1 = -40/100+x1; 
    xf2 = 40/100+x2; xb2 = -40/100+x2;
    xf3 = 40/100+x3; xb3 = -40/100+x3;
    xf4 = 40/100+x4; xb4 =-40/100+x4 ;
    [qcycle1Afrente,k] = via(xf1,xb1,y1,zu,zd,leg1Af);
    qcycle1Atras = via(xf2,xb2,y2,zu,zd,leg1At);
    qcycle2Afrente = via(xf3,xb3,y3,zu,zd,leg2Af);
    qcycle2Atras = via(xf4,xb4,y4,zu,zd,leg2At);
end
%%
function [qcycle,k] = via(xf,xb,y,zu,zd,leg)
    auxVia = [xf y zd
        xb y zd
        xb y zu
        xf y zu
        xf y zd
    ];
    location = mstraj(auxVia,[],[3 0.25 0.5 0.25],[],0.01,0.1);
    k = viaTrajectoire(location,zd);
    if(mod(length(location),2)~=0)
    location(length(location),:) = [];
    end
    qcycle = ikineTrajNum(leg,se3(eye(3),location),"link8",weights = [0 0 0 1 1 1]);
    
end
%%
function [k] = viaTrajectoire(loc,zd)
    for k = 1:length(loc) 
        if (loc(k,3) > zd) 
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
function [ang1,ang2,ang3,ang4] = jointAngles(cycleSize,o1,o2,o3,o4,if1Afrente,...
    if1Atras,if2Afrente,if2Atras,i)
        gait1 = mod(i + o1-1,cycleSize)+1;
        gait2 = mod(i + o2-1,cycleSize)+1;
        gait3 = mod(i + o3-1,cycleSize)+1;
        gait4 = mod(i + o4-1,cycleSize)+1;
        ang1 = rad2deg(if1Afrente(gait1,:));
        ang2 = rad2deg(if2Afrente(gait2,:));
        ang3 = rad2deg(if1Atras(gait3,:));
        ang4 = rad2deg(if2Atras(gait4,:));
        fprintf("\n ang1_q1: %.4f \n ang1_q2: %.4f \n ang1_q3: %.4f" + ...
            "\n ang2_q1: %.4f\n ang2_q2: %.4f\n ang2_q3: %.4f\n ang3_q1: %.4f\n" + ...
            " ang3_q2: %.4f\n ang3_q3: %.4f\n ang4_q1: %.4f\n ang4_q2: %.4f\n ang4_q3: %.4f\n",ang1(1,1)...
        ,ang1(1,2),ang1(1,3),ang2(1,1),ang2(1,2),ang2(1,3),ang3(1,1),ang3(1,2) ...
        ,ang3(1,3),ang4(1,1),ang4(1,2),ang4(1,3));
end
