% Positioning task
clear
close all
clc

try
    fclose(instrfindall);
catch
end

%% Definindo o Rob�
P = Pioneer3DX;
P.pPar.a = 0.3;
P.pPar.alpha = 0*pi/3;
% P.rConnect; % Descomentar para realiaza��o de experimento
P.rSetPose
% P.rSetPose([0 0 0 0]);

V = VREP;
V.vConnect;
V.vHandle('Pioneer_p3dx');

%% Definindo controlador
J = JoyControl;
J.pSNES = 1;

%% Definindo a Figura que ir� rodar a simula��o
figure(1)
hold on
grid on
title('Simula��o');
axis([-5 5 -5 5])
P.mCADplot2D('r')   % Visualiza��o 2D
drawnow

% Tempo de esperar para in�cio do experimento/simula��o
dados = [];
pause(1)

%% Tempo Real
tmax = 120;
tc = tic;
tp = tic;
t = tic;

%% Autonomy switch: 
disp('AutoPilot is OFF!')
AutoPilotON = 0;  
gamma = 0.01; ts = tic; %amortecimento 
plus = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);
%% Inicio da simula��o
while (toc(t) <= tmax)
    % Sub-rotina de controle
    if toc(tc) > 0.1
        % Inicio da realimenta��o
        tc = tic;
        
        P.pPos.Xda = P.pPos.Xd;
        
        % Ler comandos
        V.vGetSensorData(P,1);
                
        [P,tmax,AutoPilotON] = pSafetyControl(P,J,AutoPilotON,tmax);
        P = fKinematicControllerExtended(P);
        
        % Armazenar dados da simula��o
        dados = [dados; [P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(t)]];
        
        % Enviar sinais de controle para o rob�      
%         V.vSendControlSignals(P,1);
        P.rSendControlSignals;
        % Sub-rotina para plotar
        if toc(tp) > 0.1
            if (AutoPilotON)
                delete(plus)
                plus = plot(P.pPos.Xd(1),P.pPos.Xd(2),'r+','MarkerSize',7,'LineWidth',1.2);
            end
            
            tp = tic;
            % Plot da simula��o
            P.mCADdel
            % P.mCADplot(1,'r') % Visualiza��o 3D
            P.mCADplot2D('r')   % Visualiza��o 2D
            drawnow
        end
    end
end
disp('End of simulation!')
P.pSC.Ud([1 2]) = 0;
P.rSendControlSignals;
V.vSendControlSignals(P,1);

