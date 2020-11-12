function mLerDadosSensores(obj)

if obj.pStatus.Conectado
    
%     % Por integra��o num�ricas
%     % Posi��o do centro do rob�
%     obj.pPos.Xp(1) = arrobot_getx/1000;
%     obj.pPos.Xp(2) = arrobot_gety/1000;
%     obj.pPos.Xp(6) = arrobot_getth/180*pi;
%     
%     obj.pSC.Ua   = obj.pSC.U;
%     obj.pSC.U(1) = arrobot_getvel/1000;
%     obj.pSC.U(2) = arrobot_getrotvel/180*pi;
%     
%    % Posi��o rob� por integra��o
%     obj.pPos.Xs(6) = obj.pPos.Xp(6);
%     obj.pPos.Xs([1 2 6]) = obj.pPos.Xs([1 2 6]) + ...
%         [cos(obj.pPos.Xs(6)) -obj.pPar.a*sin(obj.pPos.Xs(6)); ...
%         sin(obj.pPos.Xs(6)) obj.pPar.a*cos(obj.pPos.Xs(6)); 0 1]*obj.pSC.U*0.1;
%     obj.pPos.Xs(6) = obj.pPos.Xp(6);  
%     obj.pPos.Xs([7 8 12]) = [cos(obj.pPos.Xs(6)) -obj.pPar.a*sin(obj.pPos.Xs(6)); ...
%         sin(obj.pPos.Xs(6)) obj.pPar.a*cos(obj.pPos.Xs(6)); 0 1]*obj.pSC.U;  
%     
%     if abs(obj.pPos.Xs) > pi
%         if obj.pPos.Xs > 0
%             obj.pPos.Xs = -2*pi + obj.pPos.Xs;
%         else
%             obj.pPos.Xs = 2*pi + obj.pPos.Xs;
%         end
%     end
%     
%     % Posi��o do centro do rob�s
%     obj.pPos.Xc([1 2 6]) = obj.pPos.Xs([1 2 6]) - ...
%         [obj.pPar.a*cos(obj.pPos.Xs(6)); obj.pPar.a*sin(obj.pPos.Xs(6)); 0];   
    
    % --------------------------------------------------------------------
    % Busca pelo ponto central
    % Posi��o do centro do rob�
    obj.pPos.Xc(1) = arrobot_getx/1000;
    obj.pPos.Xc(2) = arrobot_gety/1000;
    obj.pPos.Xc(6) = arrobot_getth/180*pi;
    
    obj.pSC.Ua   = obj.pSC.U;
    obj.pSC.U(1) = arrobot_getvel/1000;
    obj.pSC.U(2) = arrobot_getrotvel/180*pi;
    
    K1 = [ cos(obj.pPos.Xc(6)), 0; ...
        sin(obj.pPos.Xc(6)), 0];
    
    K2 = [ cos(obj.pPos.Xc(6)), -obj.pPar.a*sin(obj.pPos.Xc(6)); ...
        sin(obj.pPos.Xc(6)), +obj.pPar.a*cos(obj.pPos.Xc(6))];

    obj.pPos.Xc(7:8) = K1 * obj.pSC.U;
    obj.pPos.Xc(12) = obj.pSC.U(2);
    
    % Posi��o de controle do rob�s
    obj.pPos.Xs([1 2 3]) = obj.pPos.Xc([1 2 3]) + [obj.pPar.a*cos(obj.pPos.Xc(6)); obj.pPar.a*sin(obj.pPos.Xc(6)); 0];
    obj.pPos.Xs([4 5 6]) = obj.pPos.Xc([4 5 6]); 
    obj.pPos.Xs(7:8) = K2 * obj.pSC.U;
    obj.pPos.Xs(12) = obj.pSC.U(2);
    
    
    obj.pPos.X = obj.pPos.Xs;
else
    obj.pPos.Xs = obj.pPos.X;
    
    % Posi��o do centro do rob�s
    obj.pPos.Xc([1 2 6]) = obj.pPos.X([1 2 6]) - [obj.pPar.a*cos(obj.pPos.Xs(6)); obj.pPar.a*sin(obj.pPos.Xs(6)); 0];   
    
end

