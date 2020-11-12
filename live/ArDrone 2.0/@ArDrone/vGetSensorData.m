%% Fun��o para pegar as informa��es de Velocidade, Posi��o e Postura

function vGetSensorData(drone)

%% Posi��o do GPS acoplado ao Pioneer
[erro,drone.pPos.X(1:3,1)]= drone.vrep.simxGetObjectPosition(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_buffer);


%% Orienta��o do Pioneer
[erro,drone.pPos.X(4:6,1)] = drone.vrep.simxGetObjectOrientation(drone.clientID,drone.DroneHandle,-1,drone.vrep.simx_opmode_buffer);


% alpha, beta e gama

%% Velocidade do Pioneer
[erro,drone.pPos.X(7:9,1), drone.pPos.X(10:12,1)]= drone.vrep.simxGetObjectVelocity(drone.clientID,drone.DroneHandle,drone.vrep.simx_opmode_buffer);




end