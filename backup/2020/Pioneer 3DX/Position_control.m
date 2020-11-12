%% POSITION CONTROL FOR PIONEER
% (A pedido do Prof. Sarcinelli)

clear
close all
clc
% Fecha todas poss�veis conex�es abertas
try
    fclose(instrfindall);
catch
end

%% Look for root directory
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Open file to save data
NomeArq = datestr(now,30);
cd('DataFiles')
cd('Log_Position_Pioneer')
Arq = fopen(['PositionMobileSim_' NomeArq '.txt'],'w');
cd(PastaAtual)

%% Defini��o da janela do gr�fico
fig = figure(1);
axis([-4 4 -4 4])
axis equal

%% Classes initialization
% Robot
P = Pioneer3DX;
P.pPar.a     = -.5;     % control point
P.pPar.alpha =  0;     % control point angle

%% Conex�o com rob�/simulador
% P.rConnect;             % rob� ou mobilesim
% pause(10);
disp('Start...............')

%% Posi��o inicial do rob�
Xo = [0 0 0 0];
P.rSetPose([0 0 0 0]);    % define pose do rob�
P.pPos.X(1:2) = [0 0];

% Desired pose [x y]
P.pPos.Xd(1:2) = [2 3];
% P.pPos.Xd(1:2) = [3 1];


%% Initial errors
P.pPos.Xtil = P.pPos.Xd - P.pPos.X;

rho   =  sqrt(P.pPos.Xtil(1)^2 + P.pPos.Xtil(2)^2);
theta = atan2(P.pPos.Xtil(2),P.pPos.Xtil(1));
alfa  = theta - P.pPos.X(6);

%% Variables initialization
vmax = .75;   % pioneer maximum speed
kw   = 0.7;     % angular velocity gain
XX   = [];    % simulation data

%% Simulation

% Temporiza��o
tap = 0.1;     % taxa de atualiza��o do pioneer
tc = tic;
tp = tic;
t  = tic;      % simulation time

while rho > 0.1
    
       
    if toc(tc) > tap
        
        tc = tic;    % reset clock
        
        % ---------------------------------------------------------------
        % Data aquisition
        P.rGetSensorData;
        
        % --------------------------------------------------------------
        % Errors
        P.pPos.Xtil = P.pPos.Xd - P.pPos.X;

        rho   = sqrt(P.pPos.Xtil(1)^2 + P.pPos.Xtil(2)^2);
        theta = atan2(P.pPos.Xtil(2),P.pPos.Xtil(1));
        alfa  = theta - P.pPos.X(6);

        % --------------------------------------------------------------
        % Control
        P = fKinematicControllerExtended(P);        % new controller (by timotiu 2020)        
%         P.pSC.Ud(1) = vmax*tanh(rho)*cos(alfa);
%         P.pSC.Ud(2) = kw*alfa + vmax*tanh(rho)*sin(alfa)*cos(alfa)/rho;
        % -------------------------------------------------------------
%         P.pSC.Ud
        % Send control to robot
        P.rSendControlSignals;
        
        % Save data 
        XX = [XX; P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' rho alfa theta toc(t)];
        
        % Txt file
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' rho alfa theta toc(t)]);
        fprintf(Arq,'\n\r');
        
    end
    
    %% Desenha os rob�s
    
    if toc(tp) > tap
        tp = tic;
        %         try
        P.mCADdel
        delete(fig);
        P.mCADplot2D('r');
%         P.mCADplot(1,'k')
        plot(XX(:,1),XX(:,2),'kx');
        hold on;
        plot(XX(:,13),XX(:,14),'g');
        axis([-4 4 -4 4])
        grid on
        drawnow
    end
    
end

% Close txt file
fclose(Arq);

%%  Stop robot
% Zera velocidades do rob�
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;

%% Results

% Position
figure;
plot(XX(:,1),XX(:,2),'kx','MarkerSize',10),hold on;
plot(XX(:,13),XX(:,14)), legend('Desejado','Percorrido')
grid on
% Errors
figure;
plot(XX(:,end),XX(:,29))
title('\rho [m]')
grid on

figure;
plot(XX(:,end),rad2deg(XX(:,30)))
title('\alpha [^o]')
grid on


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
