%% Line reactive navigation v2 (03/02/2020)
%% Boas pr�ticas
close all
clearvars
clc

%% Carregando os objetos do cen�rio
try
    % Load Classes
    
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
%     % Inicializando o OptiTrack
    OPT = OptiTrack;    % Criando o OptiTrack
    OPT.Initialize;     % Iniciando o OptiTrack
    
    P = RPioneer(1,'P1'); % P�oneer3DX Experimento
    idP = 1;
    P.rDisableMotors;
    
    % Joystick
    J = JoyControl;
    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end   

subLaser = rossubscriber('/scan','sensor_msgs/LaserScan');
Laser = [];
%%
for ii = 1:100
    Laser = subLaser.LatestMessage;
end

Laser = subLaser.LatestMessage;
LaserD = Laser.readCartesian;

%% Definindo o Rob�
% P = Pioneer3DX;
P.pPar.a = 0; %0.3
P.pPar.alpha = 0;

figure(2)
hold on
grid on
title('Experimento');
axis([-5 5 -5 5])
P.mCADplot2D('r')   % Visualiza��o 2D
drawnow
% Inicializando vari�veis para o controle
%Declarando o trace:
dados=[];
P.rGetSensorData;
P.rEnableMotors;

%Par�metros Line
LinMap=zeros(8,2); Map=[]; hmed = []; Hist = [];
tp = tic;
% Rotina da simula��o:
t=tic;  ta=tic; tL = tic; tmax=120;
% tmax=80; %Percurso todo
% k1=0.6; k2=0.4; it = 0;
J = JoyControl; J.pSNES = 0; Pilot = 0;
% plot(LaserD(:,1),LaserD(:,2),'r+','MarkerSize',7,'LineWidth',1.2);
while toc(t)<tmax
    % Get Laser Data
    if toc(tL)>0.5
        tL = tic;
        %Line:
        Laser = subLaser.LatestMessage;
        Map = Laser.readCartesian;
    end
    
    % Robot control
    if toc(ta)>0.1
        ta=tic;         
        
        if isempty(Map)
            % Decide later
        else
            %Subsampling:
            for kk = 20:20:size(Map(:,2),1)
                X = Map(kk,1);
                Y = Map(kk,2);

                %Angular coeff:
                b = X\Y;
                Ylinha = X*b;

                LinMap(kk/20,1) = X(1);
                LinMap(kk/20,2) = Ylinha(1);
            end
            
            XX = LinMap(:,1)+ P.pPos.X(1);
            YY = LinMap(:,2)+ P.pPos.X(2);
            b1 = XX\YY;
            Yreta = XX*b1;

            med = mean(LinMap);
 
            figure(2)
            hold on
            plus = plot(med(1),med(2),'r+','MarkerSize',7,'LineWidth',1.2);
            hmed = [hmed;med];
            
            %Posi��o do ponto m�dio:
            P.pPos.Xd(1)= med(1);
            P.pPos.Xd(2)= med(2);
            P.pPos.Xa(1:2) = med(1:2);
                
            figure(2)
            p1=plot(LinMap(:,1),LinMap(:,2),'b--');
            drawnow
        end

        if ~isempty(hmed) % Se o mapa estiver vazio
                P.pPos.Xd(1)= med(1);
                P.pPos.Xd(2)= med(2);
                P.pPos.Xa(1:2) = med(1:2);
        end

        %Pegar informa��o da posi��o e velocidade real do rob�
        P.rGetSensorData;
        
        % Planejamento:
        if Pilot ~= 2
            P.pPos.X(6)= atan2(P.pPos.X(2)-P.pPos.Xc(2),P.pPos.X(1)-P.pPos.Xc(1));
            P.pPos.Xd(7:8)= (P.pPos.Xd(1:2)-P.pPos.Xa(1:2))/toc(ta);
        end
        
        % Controle:        
%         [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax);
        P = fKinematicControllerExtended(P,[0.13 0.13 1]);
       
        % Armazenar dados da simula��o
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
                                  
        % Enviar sinais de controle para o rob�
        disp(P.pSC.Ud)
        P.pSC.Ud = P.pSC.Ud.*0.1; 
        P = J.mControl(P);
        J.mRead;
        if J.pDigital(1) || J.pDigital(2)
            tmax = 0;
        end
%         P.rCommand;       
        Hist = [P.pPos.Xc,Hist];
        
        P.pSC.Ud([1 2]) = 0;
        % Sub-rotina para plotar
        figure(2)
        if toc(tp) > 0.1
            if (Pilot == 1)
                delete(plus)
                plus = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);
            end
            
            tp = tic;
            % Plot da simula��o
            P.mCADdel
            P.mCADplot2D('r')   % Visualiza��o 2D
            drawnow
        end
        
    end
end

% if Pilot == 0
%     disp('Press START to quit.')
%     J.pDigital(5) = 1;
%     while ~J.pDigital(end)
%         if toc(ta)>0.1
%             ta=tic;
%             P.rGetSensorData; 
%             [P,tmax,Pilot] = pSafetyControl(P,J,Pilot,tmax);
%             P = fKinematicControllerExtended(P);
%             P.rCommand;  
%         end
%     end
% end

P.pSC.Ud([1 2]) = 0;
P.rCommand;
P.rDisableMotors;
%% Desconecta Matlab e V-REP
% P.rDisconnect;

