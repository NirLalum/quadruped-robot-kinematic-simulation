%% main
clear all
clc

global l1 l2 l3 l4 w H CM_MEC_OLD CM_MEC_NEW AuxVec safetyFactor 
l1 = 6.63;
l2 = 8.57;
l3 = 26;
l4 = 26;
w = 22.8; % robot width 
H = 52.8; % robot length
CM_MEC_OLD = [0 ,0, 0];
CM_MEC_NEW = [0 ,0, 0];  
AuxVec = CM_MEC_NEW - CM_MEC_OLD;
safetyFactor = 3;
forwardDelta = 30; % how much to go foraward relative to robot system
crabDelta = 10;  % how much to go to the right relative to robot system
spinDelta = pi/10; % how much to go turn relative to robot system
A_R_W = getRobotToWorldTrans(0, 0, 0, 0); % initial A_R_W
currGamma = 0; % initial gamma 

indicator = input('where to go? \n', 's');
while double(indicator) ~= double('t') % t (116 in ascii) for terminate
    indicatorVal = double(indicator); 
    switch indicatorVal
        case 'w' % forward
            forWardInit = A_R_W(1:3, 4); 
            forWardFin = A_R_W * [0 forwardDelta 0 1]';
            A_R_W = robotForward(currGamma, forWardInit(1), forWardInit(2), forWardInit(3), currGamma, forWardFin(1), forWardFin(2), forWardFin(3), 1);
        case 's' % backward
            backWardInit = A_R_W(1:3, 4); 
            backWardFin = A_R_W * [0 -forwardDelta 0 1]';
            A_R_W = robotForward(currGamma, backWardInit(1), backWardInit(2), backWardInit(3), currGamma, backWardFin(1), backWardFin(2), backWardFin(3), -1);
        case 'a' % spin +z
            currentCOMloc = A_R_W(1:3, 4);
            A_R_W = robotTurn(currGamma , currGamma + spinDelta, currentCOMloc(1), currentCOMloc(2), currentCOMloc(3));
            currGamma = currGamma + spinDelta;
        case 'd' % spin -z
            currentCOMloc = A_R_W(1:3, 4);
            A_R_W = robotTurn(currGamma , currGamma - spinDelta, currentCOMloc(1), currentCOMloc(2), currentCOMloc(3));
            currGamma = currGamma - spinDelta;
        case 'e' % crab walk to the rigth
            crabRightInit = A_R_W(1:3, 4); 
            crabRightFin = A_R_W * [crabDelta 0 0 1]';
            A_R_W = crabWalk(currGamma, crabRightInit(1), crabRightInit(2), crabRightInit(3), currGamma, crabRightFin(1), crabRightFin(2), crabRightFin(3), 1); 
        case 'q' % crab walk to the left
            crabRightInit = A_R_W(1:3, 4); 
            crabRightFin = A_R_W * [-crabDelta 0 0 1]';
            A_R_W = crabWalk(currGamma, crabRightInit(1), crabRightInit(2), crabRightInit(3), currGamma, crabRightFin(1), crabRightFin(2), crabRightFin(3), -1); 
    end
    indicator = input('where to go? \n', 's');
end

%% this function makes one full step of the whole robot (forward/backwards)
% backwards is 1 for forward movement and -1 for backwards movement
function A_R_W = robotForward(gamma_init, cmX_init, cmY_init, cmZ_init,gamma_des,cmX_des, cmY_des, cmZ_des, backwards) 
    
    global l2 
    % robot body sizes and 
    
    xi = l2; yi = 30; zi = 0; % relative to the leg
    tiltDelta = 8; % define the tilt of the robot when rising a leg
    % leg to rom com matrix
    A_R_0_lf = getLegToRobotTrans(1);
    A_R_0_rf = getLegToRobotTrans(2);
    A_R_0_rb = getLegToRobotTrans(3);
    A_R_0_lb = getLegToRobotTrans(4);
    % define where not moving legs should be
    staticPosLeg1 = [-xi, yi , zi]; % staticPos is defined in each leg axis and saves the position of the legs in each iteration
    staticPosLeg2 = [xi, yi , zi];
    staticPosLeg3 = [-xi, yi , zi];
    staticPosLeg4 = [xi, yi , zi];
    leg_delta = norm([cmX_des, cmY_des, cmZ_des] - [cmX_init, cmY_init, cmZ_init]); % leg's displacment
    [xVec1, yVec1, zVec1] = LegPath(0, backwards*leg_delta, 1, 0,xi, 1); % zinit, zfin, leg, xinit, xfin, spin or linear. all relative to the leg
    [xVec2, yVec2, zVec2] = LegPath(0, backwards*leg_delta, 2, 0,xi, 1);
    [xVec3, yVec3, zVec3] = LegPath(0, backwards*leg_delta, 3, 0,xi, 1);
    [xVec4, yVec4, zVec4] = LegPath(0, backwards*leg_delta, 4, 0,xi, 1);
    pathCell = {[xVec1; yVec1; zVec1], [xVec2; yVec2; zVec2], [xVec3; yVec3; zVec3], [xVec4; yVec4; zVec4]};
    staticMat = [staticPosLeg1; staticPosLeg2; staticPosLeg3; staticPosLeg4];  
    A_R_W = getRobotToWorldTrans(gamma_init, cmX_init, cmY_init, cmZ_init); % robot to world transform
    legSequance = [1, 2, 4, 3, 5]; % 5 is for the last tilt
    if backwards == -1
        legSequance = [4, 3, 1, 2, 5];
    end
    for leg = legSequance        
        switch leg
            case legSequance(1)
                % tilt COM to the right
                tiltInWorldCords = A_R_W * [tiltDelta 0 0 1]'; % [tiltDelta 0 0 1] tilting in robot system
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(cmX_init, cmY_init, cmZ_init, gamma_init, tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gamma_des);
                 last_A_R_W = A_R_W;
   
                 [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm , zcm, gammaVec,staticMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                 A_R_W = getRobotToWorldTrans(gamma_des,tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3));
                 staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];
                 [pathCell{1, leg}(1, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(3, :)] = LegPath( 0, backwards*leg_delta, leg, 0, xi + tiltDelta, 1);
            
            case legSequance(2)
                % bring back the com to initial pos
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gamma_init, cmX_init, cmY_init, cmZ_init, gamma_des);
                 last_A_R_W = A_R_W;
   
                 [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm , zcm, gammaVec,staticMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                 A_R_W = getRobotToWorldTrans(gamma_des,cmX_init, cmY_init, cmZ_init);
                 staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];  
               
            case legSequance(3) % determines when the COM push starts      
                % push COM forward/backwards 
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(cmX_init, cmY_init, cmZ_init, gamma_init,cmX_des, cmY_des, cmZ_des, gamma_des); % create a path to the COM
                 
                 last_A_R_W = A_R_W;
   
                 [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm, zcm, gammaVec,staticMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                 A_R_W = getRobotToWorldTrans(gamma_des,cmX_des, cmY_des, cmZ_des); 
                 % update leg path and leg position after cm movement
                 [pathCell{1, 1}(1, :) , pathCell{1,1}(2, :), pathCell{1,1}(3, :)] = LegPath(-backwards*leg_delta, 0, 1, 0, xi, 1);
                 [pathCell{1, 2}(1, :) , pathCell{1,2}(2, :), pathCell{1,2}(3, :)] = LegPath(-backwards*leg_delta, 0, 2, 0, xi, 1);
                 [pathCell{1, 3}(1, :) , pathCell{1,3}(2, :), pathCell{1,3}(3, :)] = LegPath(-backwards*leg_delta, 0, 3, 0, xi, 1);
                 [pathCell{1, 4}(1, :) , pathCell{1,4}(2, :), pathCell{1,4}(3, :)] = LegPath(-backwards*leg_delta, 0, 4, 0, xi, 1);
                 staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];
            
            case legSequance(4)
                % tilt COM to the left
                tiltInWorldCords = A_R_W * [-tiltDelta 0 0 1]'; % [tiltDelta 0 0 1] tilting in robot system
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(cmX_des, cmY_des, cmZ_des, gamma_init,tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gamma_des);
                 last_A_R_W = A_R_W;
   
                 [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm , zcm, gammaVec,staticMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                 A_R_W = getRobotToWorldTrans(gamma_des, tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3));
                 staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];
                [pathCell{1,leg}(1, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(3, :)] = LegPath(-backwards*leg_delta, 0,leg, 0, xi + tiltDelta, 1);
            case legSequance(5)
                % tilt COM back to the center
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gamma_init,cmX_des, cmY_des, cmZ_des, gamma_des);
                 last_A_R_W = A_R_W;
   
                 moveCOM(xcm, ycm , zcm, gammaVec,staticMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);        
                  A_R_W = getRobotToWorldTrans(gamma_des, cmX_des, cmY_des, cmZ_des);
        end
        if leg == legSequance(5) % finish the cycle
            return
        end    

        staticMat = moveLeg(staticMat, pathCell,A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb, leg);
    end
end

%%
function A_R_W = robotTurn(gammaInit, gammaFin, currCMx, currCMy, currCMz)
    global l2
    
    % for counter clockwise:
    aux1 = 1; aux2 = -1;
    %if counter clockwise
    legSequance = [1, 4, 3, 2, 5]; 
     if gammaFin < gammaInit
         aux1 = -1; aux2 = 1;
%         legSequance = [2, 3, 4, 1, 5]; 
     end
    
    
    % robot body sizes and 
    xi = l2; yi = 30; zi = 0; % relative to the leg
    tiltDelta = 8; % define the tilt of the robot when rising a leg
    % leg to rom com matrix
    A_R_0_lf = getLegToRobotTrans(1);
    A_R_0_rf = getLegToRobotTrans(2);
    A_R_0_rb = getLegToRobotTrans(3);
    A_R_0_lb = getLegToRobotTrans(4);
    % define where not moving legs should be at the beginning of the cycle
    staticPosLeg1 = [-xi, yi , zi]; % staticPos is defined in each leg axis 
    staticPosLeg2 = [xi, yi , zi];
    staticPosLeg3 = [-xi, yi , zi];
    staticPosLeg4 = [xi, yi , zi];
    [xReq13, zReq13] = getXYZforSpin(abs(abs(gammaFin) - abs(gammaInit)), xi, zi, 1); % x and y required for legs 1 and 3 relative to the robot leg
    [xReq24, zReq24] = getXYZforSpin(abs(abs(gammaFin) - abs(gammaInit)), xi, zi, 2); % x and y required for legs 2 and 4 relative to the robot leg 
    % in case the spin is in -z direction
    if gammaFin < gammaInit 
        temp = xReq13; 
        xReq13 = xReq24;
        xReq24 = temp;
        temp = zReq13;
        zReq13 = zReq24;
        zReq24 = temp;
    end
    
    %leg_delta = norm([xReq, yReq]);
    [xVec1, yVec1, zVec1] = LegPath(0, aux2*zReq13, 1, xi, aux1*xReq13, 0); % zinit, zfin, leg relative to the leg
    [xVec2, yVec2, zVec2] = LegPath(0, aux1*zReq24, 2, xi, aux2*xReq24, 0);
    [xVec3, yVec3, zVec3] = LegPath(0, aux1*zReq13, 3, xi, aux1*xReq13, 0);
    [xVec4, yVec4, zVec4] = LegPath(0, aux2*zReq24, 4, xi, aux2*xReq24, 0);
    pathCell = {[xVec1; yVec1; zVec1], [xVec2; yVec2; zVec2], [xVec3; yVec3; zVec3], [xVec4; yVec4; zVec4]};
    staticMat = [staticPosLeg1; staticPosLeg2; staticPosLeg3; staticPosLeg4];  
    A_R_W = getRobotToWorldTrans(gammaInit, currCMx, currCMy, currCMz); % robot to world transform
    
    for leg = legSequance        
        switch leg
            case legSequance(1)
                % tilt COM to the right
                tiltInWorldCords = A_R_W * [tiltDelta 0 0 1]'; % [tiltDelta 0 0 1] tilting in robot system
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(currCMx, currCMy, currCMz, gammaInit, tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gammaInit);
                last_A_R_W = A_R_W;

                [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm , zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                A_R_W = getRobotToWorldTrans(gammaInit,tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3));
                staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];
                [pathCell{1, leg}(1, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(3, :)] = LegPath( 0, aux2*zReq13, 1, xi + tiltDelta, aux1*xReq13 + tiltDelta, 0);

                
            case legSequance(2)

                [pathCell{1, leg}(1, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(3, :)] = LegPath(0, aux2*zReq24, 4, xi + tiltDelta, aux2*xReq24 + tiltDelta, 0);
   
            case legSequance(3)                   
                % tilt from right to left
                tiltInWorldCords2 = A_R_W * [-2*tiltDelta 0 0 1]';
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gammaInit, tiltInWorldCords2(1), tiltInWorldCords2(2), tiltInWorldCords2(3), gammaInit);
                last_A_R_W = A_R_W;

                [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm , zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                A_R_W = getRobotToWorldTrans(gammaInit,tiltInWorldCords2(1), tiltInWorldCords2(2), tiltInWorldCords2(3));
                staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];
                [pathCell{1, leg}(1, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(3, :)] = LegPath( 0, aux1*zReq13, 3, xi + tiltDelta, aux1*xReq13 + tiltDelta, 0);

                    
            case legSequance(4)
                % update leg Path because the original is relative to the
                % robot standing in the middle
                [pathCell{1, leg}(1, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(3, :)] = LegPath(0, aux1*zReq24, 2, xi + tiltDelta, aux2*xReq24 + tiltDelta, 0);
                
                
            case legSequance(5)
                % back to center
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(tiltInWorldCords2(1), tiltInWorldCords2(2), tiltInWorldCords2(3), gammaInit, currCMx, currCMy, currCMz, gammaInit);
                last_A_R_W = A_R_W;

                [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm , zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                A_R_W = getRobotToWorldTrans(gammaInit, currCMx, currCMy, currCMz);
                staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)]; 
                
                
                % spin
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(currCMx, currCMy, currCMz, gammaInit,currCMx, currCMy, currCMz, gammaFin); % create a path to the COM
                 
                 last_A_R_W = A_R_W;
   
                 moveCOM(xcm, ycm, zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                 A_R_W = getRobotToWorldTrans(gammaFin, currCMx, currCMy, currCMz); % for next step  
        end
        if leg == legSequance(5)
            return
        end 
        staticMat = moveLeg(staticMat, pathCell,A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb, leg);
    end
end



%% 
% side is 1 for right and -1 for left
function A_R_W = crabWalk(gamma_init, cmX_init, cmY_init, cmZ_init,gamma_des,cmX_des, cmY_des, cmZ_des, side) 
    global l2 
    % robot body sizes and 
    xi = l2; yi = 30; zi = 0; % relative to the leg
    tiltDelta = 8; % define the tilt of the robot when rising a leg
    % leg to rom com matrix
    A_R_0_lf = getLegToRobotTrans(1);
    A_R_0_rf = getLegToRobotTrans(2);
    A_R_0_rb = getLegToRobotTrans(3);
    A_R_0_lb = getLegToRobotTrans(4);
    % define where not moving legs should be
    staticPosLeg1 = [-xi, yi , zi]; % staticPos is defined in each leg axis and saves the position of the legs in each iteration
    staticPosLeg2 = [xi, yi , zi];
    staticPosLeg3 = [-xi, yi , zi];
    staticPosLeg4 = [xi, yi , zi];
    leg_delta = norm([cmX_des, cmY_des, cmZ_des] - [cmX_init, cmY_init, cmZ_init]); % leg's displacment
    [zVec1, yVec1, xVec1] = LegPath(xi, side*(xi+leg_delta), 1, 0, 0, 1); % xinit, xfin, leg, zinit, zfin, spin or linear. all relative to the leg
    [zVec2, yVec2, xVec2] = LegPath(xi, side*(xi+leg_delta), 2, 0,0, 1);
    [zVec3, yVec3, xVec3] = LegPath(xi, side*(xi+leg_delta), 3, 0,0, 1);
    [zVec4, yVec4, xVec4] = LegPath(xi, side*(xi+leg_delta), 4, 0,0, 1);
    pathCell = {[xVec1; yVec1; zVec1], [xVec2; yVec2; zVec2], [xVec3; yVec3; zVec3], [xVec4; yVec4; zVec4]};
    staticMat = [staticPosLeg1; staticPosLeg2; staticPosLeg3; staticPosLeg4];  
    A_R_W = getRobotToWorldTrans(gamma_init, cmX_init, cmY_init, cmZ_init); % robot to world transform
    legSequance = [2, 3, 1, 4, 5]; % 5 is for the last tilt
    if side == -1
        legSequance = [1, 4, 2, 3, 5];
    end
    for leg = legSequance        
        switch leg
            case legSequance(1)
                % tilt COM to the right/left (depends on which side the crab walk is)
                tiltInWorldCords = A_R_W * [-side*tiltDelta 0 0 1]'; % [tiltDelta 0 0 1] tilting in robot system
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(cmX_init, cmY_init, cmZ_init, gamma_init, tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gamma_des);
                 last_A_R_W = A_R_W;
   
                 [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm , zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                 A_R_W = getRobotToWorldTrans(gamma_des,tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3));
                 staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];
                 [pathCell{1, leg}(3, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(1, :)] = LegPath( side*(xi + tiltDelta), side*(xi+leg_delta + tiltDelta), leg, 0, 0, 1);
            
            case legSequance(2)
                [pathCell{1, leg}(3, :) , pathCell{1, leg}(2, :), pathCell{1,leg}(1, :)] = LegPath( side*(xi + tiltDelta), side*(xi+leg_delta + tiltDelta), leg, 0, 0, 1);
               
            case legSequance(3)       
                % push COM to the other side
                tiltInWorldCords2 = A_R_W * [side*(leg_delta + 2*tiltDelta) 0 0 1]';
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(tiltInWorldCords(1), tiltInWorldCords(2), tiltInWorldCords(3), gamma_init, tiltInWorldCords2(1), tiltInWorldCords2(2), tiltInWorldCords2(3), gamma_des); % create a path to the COM
                 
                 last_A_R_W = A_R_W;
   
                 [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm, zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                 A_R_W = getRobotToWorldTrans(gamma_des, tiltInWorldCords2(1), tiltInWorldCords2(2), tiltInWorldCords2(3)); 
                 % update leg path and leg position after cm movement
                 [pathCell{1, leg}(3, :) , pathCell{1,leg}(2, :), pathCell{1,leg}(1, :)] = LegPath(side*(-xi - (leg_delta + tiltDelta)), -side*(xi + tiltDelta), leg, 0, 0, 1);

                 staticMat = [staticPosLeg1(1:3); staticPosLeg2(1:3); staticPosLeg3(1:3); staticPosLeg4(1:3)];
            
            case legSequance(4)
                [pathCell{1, leg}(3, :) , pathCell{1,leg}(2, :), pathCell{1,leg}(1, :)] = LegPath(side*(-xi - (leg_delta + tiltDelta)), -side*(xi + tiltDelta), leg, 0, 0, 1);
           
            case legSequance(5)
                % move CM to desired final location
                [xcm, ycm, zcm, gammaVec] = CMpathPlan(tiltInWorldCords2(1), tiltInWorldCords2(2), tiltInWorldCords2(3), gamma_des, cmX_des, cmY_des, cmZ_des, gamma_des);
                last_A_R_W = A_R_W;
                moveCOM(xcm, ycm , zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb);
                A_R_W = getRobotToWorldTrans(gamma_des, cmX_des, cmY_des, cmZ_des);
        end
        if leg == legSequance(5) % finish the cycle
            return
        end    

        staticMat = moveLeg(staticMat, pathCell,A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb, leg);
    end
end


%% this function moves a leg and returns the current postion of all legs
function staticMat = moveLeg(staticMat, pathCell,A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb, leg)
    global CM_MEC_NEW
    i = 1;
        fprintf("start Leg movement:\n\n")
        for i = i:length(pathCell{1,1}(1,:))
            Robot_CM_W = A_R_W * [CM_MEC_NEW 1]';
            plot3(Robot_CM_W(1),Robot_CM_W(2),Robot_CM_W(3),'Marker','diamond');
            hold on
            %----------x, y, z is relative to the leg axis------------%
            if leg == 1
                plot_mat = fkin(pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i), leg,A_R_0_lf, A_R_W,1);
                staticMat(leg, 1:3) = [pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i)]; 
            else
                plot_mat = fkin(staticMat(1,1),staticMat(1,2),staticMat(1,3), 1,A_R_0_lf, A_R_W,0);
            end
            robotPlot(plot_mat)
            if leg == 2
                %fprintf("start RfLeg movement:\n")
                plot_mat = fkin(pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i), leg, A_R_0_rf, A_R_W,1);
                staticMat(leg, 1:3) = [pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i)];
            else
                plot_mat = fkin(staticMat(2,1),staticMat(2,2),staticMat(2,3), 2,A_R_0_rf, A_R_W,0);
            end

            robotPlot(plot_mat)
            if leg == 3
                %fprintf("start LbLeg movement:\n")
                plot_mat = fkin(pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i), leg, A_R_0_rb, A_R_W,1);
                staticMat(leg, 1:3) = [pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i)];
            else
                plot_mat = fkin(staticMat(3,1),staticMat(3,2),staticMat(3,3), 3,A_R_0_rb, A_R_W,0);
            end
            robotPlot(plot_mat)
            if leg == 4
                %fprintf("start RfLeg movement:\n")
                plot_mat = fkin(pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i), leg, A_R_0_lb, A_R_W,1);
                staticMat(leg, 1:3) = [pathCell{1, leg}(1, i) , pathCell{1, leg}(2, i), pathCell{1,leg}(3, i)];
            else
                plot_mat = fkin(staticMat(4,1),staticMat(4,2),staticMat(4,3), 4, A_R_0_lb, A_R_W,0);
            end
            robotPlot(plot_mat)
            PlotFrame(A_R_W,A_R_0_lf,A_R_0_rf,A_R_0_rb,A_R_0_lb)
            view(3)
            axis equal
            grid minor
            
            % ------------- print current legs polygons ------------
            if i ~= length(pathCell{1,1}(1,:)) % make polygon with 3 legs on the ground
                polygonsManager(staticMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb, Robot_CM_W, leg)
            end
            if (i == pathCell{1,1}(1,:)) % now make a polygon when 4 legs on the ground
                tempMat = staticMat;
                tempMat(leg, :) = [pathCell{1,leg}(1, i) , pathCell{1,leg}(2, i), pathCell{1,leg}(3, i)];  
                polygonsManager(tempMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb, Robot_CM_W, leg);
            end
            % ------------------- end of printing polygon ---------------
            hold off
            pause(0.001)
        end
    end



%%
% this function transform vectors from the robot CM to the world axis
% according to desired CM location and rotation of the robot relative to
% the world. _r are relative to the robot cm. Tra is the translation vector
% relative to the world
function A_R_O = getRobotToWorldTrans(gamma, TraX, TraY, TraZ)
A_R_O = [cos(gamma) -sin(gamma) 0 TraX; sin(gamma) cos(gamma) 0 TraY; 0 0 1 TraZ; 0 0 0 1 ];
end
%%
function A_leg_Rcm = getLegToRobotTrans(leg)
global AuxVec w H
switch leg
    case 1 
        TraVec = [-w/2, H/2, 0] - AuxVec;
        A_leg_Rcm = [1 0 0 TraVec(1); 0 0 1 TraVec(2); 0 -1 0 TraVec(3); 0 0 0 1];
    case 2     
        TraVec = [w/2, H/2, 0] - AuxVec;
        A_leg_Rcm = [1 0 0 TraVec(1); 0 0 1 TraVec(2); 0 -1 0 TraVec(3); 0 0 0 1];
    case 3
        TraVec = [w/2, -H/2, 0] - AuxVec;
        A_leg_Rcm = [-1 0 0 TraVec(1); 0 0 -1 TraVec(2); 0 -1 0 TraVec(3); 0 0 0 1];
    case 4
        TraVec = [-w/2, -H/2, 0] - AuxVec;
        A_leg_Rcm = [-1 0 0 TraVec(1); 0 0 -1 TraVec(2); 0 -1 0 TraVec(3); 0 0 0 1];
end
end
%%
function PlotFrame(A_R_W,A_R_0_lf,A_R_0_rf,A_R_0_lb,A_R_0_rb)
Origin = [0 0 0 1]';
edgelf = A_R_W*A_R_0_lf*Origin; edgerf = A_R_W*A_R_0_rf*Origin;
edgelb = A_R_W*A_R_0_rb*Origin; edgerb = A_R_W*A_R_0_lb*Origin;
body = [edgelf';edgerf';edgerb';edgelb';edgelf'];
plot3(body(:,1),body(:,2),body(:,3),'k','linewidth',15)
end
%%
function polygonsManager(LegPositionMat, A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb, Robot_CM_W, leg)
    global safetyFactor
    footXYZ_W1 = A_R_W * A_R_0_lf * [LegPositionMat(1,1), LegPositionMat(1,2), LegPositionMat(1,3) , 1]'; % transform foot pos to the world
    footXYZ_W2 = A_R_W * A_R_0_rf * [LegPositionMat(2,1), LegPositionMat(2,2), LegPositionMat(2,3) , 1]';
    footXYZ_W3 = A_R_W * A_R_0_rb * [LegPositionMat(3,1), LegPositionMat(3,2), LegPositionMat(3,3) , 1]';
    footXYZ_W4 = A_R_W * A_R_0_lb * [LegPositionMat(4,1), LegPositionMat(4,2), LegPositionMat(4,3) , 1]';
    switch leg
        case 1
        COMinside = plotPolygon(footXYZ_W2, footXYZ_W3, footXYZ_W4, footXYZ_W4, Robot_CM_W, safetyFactor, 3);
        case 2
        COMinside = plotPolygon(footXYZ_W1, footXYZ_W3, footXYZ_W4, footXYZ_W4, Robot_CM_W, safetyFactor, 3);
        case 3
        COMinside = plotPolygon(footXYZ_W1, footXYZ_W2, footXYZ_W4, footXYZ_W4, Robot_CM_W, safetyFactor, 3);
        case 4
        COMinside = plotPolygon(footXYZ_W1, footXYZ_W2, footXYZ_W3, footXYZ_W3, Robot_CM_W, safetyFactor, 3);
    end 

end

%%
function IsCMinside = plotPolygon(point1,point2,point3,point4,cmVec,factorOfSafty,legsNum) 
A = point1(1:2)'; B = point2(1:2)' ; C = point3(1:2)'; D = point4(1:2)';

if legsNum == 3
[a,b,c] = getPointsForTriangle(A,B,C, factorOfSafty);
IsCMinside = ISINSIDE(a,b,c,cmVec);
X1 = [A(1) B(1) C(1)]; X2 = [a(1) b(1) c(1)];% only for visual - not for CPP
Y1 = [A(2) B(2) C(2)] ; Y2 = [a(2) b(2) c(2)];% only for visual - not for CPP
Z1 = [point1(3) point2(3) point3(3)] ; Z2 = [point1(3) point2(3) point3(3)];% only for visual - not for CPP
K1 = [0 0.4470 0.7410]	; K2 = [0.3010 0.7450 0.9330]	;% only for visual - not for CPP
else 
[a,~,~] = getPointsForTriangle(A,D,B, factorOfSafty);% we only need a
[b,~,~] = getPointsForTriangle(B,A,C, factorOfSafty);% we only need b
[c,~,~] = getPointsForTriangle(C,D,B, factorOfSafty);% we only need c
[d,~,~] = getPointsForTriangle(D,C,A, factorOfSafty);% we only need d
IsCMinside = or(ISINSIDE(a,b,c,cmVec),ISINSIDE(d,a,c,cmVec))*1;
X1 = [A(1) B(1) C(1) D(1)]; X2 = [a(1) b(1) c(1) d(1)];% only for visual - not for CPP
Y1 = [A(2) B(2) C(2) D(2)] ; Y2 = [a(2) b(2) c(2) d(2)];% only for visual - not for CPP
Z1 = [point1(3) point2(3) point3(3) point4(3)] ; Z2 = [point1(3) point2(3) point3(3) point4(3)];% only for visual - not for CPP
K1 = [0 0.4470 0.7410]	; K2 = [0.3010 0.7450 0.9330]	;% only for visual - not for CPP
end

fill3(X1,Y1,Z1,K1) % only for visual - not for CPP
fill3(X2,Y2,Z2,K2) % % only for visual - not for CPP
plot3(cmVec(1), cmVec(2), point1(3), 'Marker','.','color',[0.6350 0.0780 0.1840],'LineWidth',20) % only for visual - not for CPP

end
%%
function [P1,P2,P3]  = getPointsForTriangle(P1,P2,P3, factorOfSafty)
A = P1(1:2); B = P2(1:2) ; C = P3(1:2);

AB = norm((B-A)); AC = norm((C-A)); BC = norm((C-B));
O = [(AB*C(1)+BC*A(1)+AC*B(1))/(AB+BC+AC) (AB*C(2)+BC*A(2)+AC*B(2))/(AB+BC+AC)]; % Circumscribed circle canter

OA = (O-A)/norm((O-A)); OB = (O-B)/norm((O-B)); OC = (O-C)/norm((O-C));
AB_n = (A-B)/norm((A-B)); AC_n = (A-C)/norm((A-C)); BC_n = (B-C)/norm((B-C));

angleA = acos(dot(AB_n,AC_n)); angleB = acos(dot(-AB_n,BC_n)); angleC = acos(dot(-AC_n,-BC_n));
 
P1 = A+factorOfSafty/sin(angleA/2)*OA; P2 = B+factorOfSafty/sin(angleB/2)*OB; P3 = C+factorOfSafty/sin(angleC/2)*OC;
end
%%

function [isInside] = ISINSIDE(P1,P2,P3,CM)
temp1 = betweenVertices(P1,P2,P3,CM);
temp2 = betweenVertices(P3,P1,P2,CM);
isInside = and(temp1,temp2)*1;
end
function [output] = betweenVertices(P1,P2,P3,CM)
p2p1 = (P2-P1)/norm(P2-P1); p3p1 = (P3-P1)/norm(P3-P1); cmp1 = (CM(1:2)'-P1)/norm(CM(1:2)'-P1);
angle1 = acos(dot(p2p1,p3p1)/(norm(p2p1)*norm(p3p1)));
angle2 = acos(dot(p2p1,cmp1)/(norm(p2p1)*norm(cmp1)));
angle3 = acos(dot(p3p1,cmp1)/(norm(p3p1)*norm(cmp1)));
if angle1 < angle2+angle3-10^-5
    output = 0;
else 
    output = 1;
end

end

%%
function Aout = Ainv(Ain)
    temp_R = Ain(1:3,1:3); 
    temp_d = Ain(1:3,4);
    Aout(1:3,1:3) = temp_R'; 
    Aout(1:3,4) = -temp_R'*temp_d;
    Aout(4,1:4) = [0 0 0 1];
end
%% full robot plot function
function robotPlot(plot_mat)
    plot3(plot_mat(1:2,1),plot_mat(1:2,2),plot_mat(1:2,3),'linewidth',6,'color',[0.9290 0.6940 0.1250])
    plot3(plot_mat(2:3,1),plot_mat(2:3,2),plot_mat(2:3,3),'linewidth',6,'color',[0.9290 0.6940 0.1250])
    plot3(plot_mat(3:4,1),plot_mat(3:4,2),plot_mat(3:4,3),'linewidth',6,'color',[0.6350 0.0780 0.1840])
    plot3(plot_mat(4:5,1),plot_mat(4:5,2),plot_mat(4:5,3),'linewidth',6,'color',[0.6350 0.0780 0.1840])
%             axis equal
            view(3)
            grid minor
%     plot3(plot_mat(:,1),plot_mat(:,2),plot_mat(:,3),'linewidth',2)

end


%% Leg path planning
% for forward moving forWard = 1, for turning forWard = 0
function [xVec, yVec, zVec] = LegPath(initial, final, leg, xInit, xFin, forWard) 
ti = 0;
tf = 0.1;

zi = initial;
zf = final; % positive value (the path is planned for leg number 2)
yMax = 25; 
yi = 30; 
yf = 30;

% fix z because of oppsite directions in legs
if leg == 3 || leg == 4
    zi = -zi; 
    zf = -zf;
end

vi = 0;  
vf = 0; % mm/sec
ai = 0;
af = 0;

posConditions = [ti zi; tf zf];
velConditions = [ti vi; tf vf];
accelConditions = [ti ai; tf af];

polyCoefs = double(getCoefs(posConditions, velConditions, accelConditions));

[zVec, yVec] = buildOutputVector(polyCoefs, ti, tf, zi, zf , yi, yf, yMax);   

if forWard == 1
    xVec = xFin*ones(1, length(yVec));  
else
    xVec = linspace(xInit, xFin, length(yVec));
end
% fix x because of oppsite directions in legs
if leg == 1 || leg == 3
    xVec = -xVec; 
end
    
end
%%
% xcm ycm zcm are relative to the world
function [staticPosLeg1, staticPosLeg2, staticPosLeg3, staticPosLeg4] = moveCOM(xcm, ycm, zcm, gammaVec,staticMat, last_A_R_W, A_R_0_lf, A_R_0_rf, A_R_0_rb, A_R_0_lb)
    global safetyFactor CM_MEC_NEW
    fprintf("start com movement:\n\n")
    for j = 2:length(ycm)
        footXYZ_W1 = last_A_R_W * A_R_0_lf * [staticMat(1,1), staticMat(1,2), staticMat(1,3) , 1]';
        footXYZ_W2 = last_A_R_W * A_R_0_rf * [staticMat(2,1), staticMat(2,2), staticMat(2,3) , 1]';
        footXYZ_W3 = last_A_R_W * A_R_0_rb * [staticMat(3,1), staticMat(3,2), staticMat(3,3) , 1]';
        footXYZ_W4 = last_A_R_W * A_R_0_lb * [staticMat(4,1), staticMat(4,2), staticMat(4,3) , 1]';

        A_R_W = getRobotToWorldTrans(gammaVec(j), xcm(j), ycm(j), zcm(j)); % desired A_R_W
        Robot_CM_W = A_R_W * [CM_MEC_NEW 1]'; % update cm relative to the world
        plot3(Robot_CM_W(1),Robot_CM_W(2),Robot_CM_W(3),'Marker','diamond')
        axis equal%%%%
        hold on


        PlotFrame(A_R_W,A_R_0_lf,A_R_0_rf,A_R_0_rb,A_R_0_lb)
        
        fprintf("LfLeg thetas:\n")
        moveCM(xcm(j), ycm(j), zcm(j), footXYZ_W1, 1, A_R_0_lf, gammaVec(j))
        fprintf("RfLeg thetas:\n")
        moveCM(xcm(j), ycm(j), zcm(j), footXYZ_W2, 2, A_R_0_rf, gammaVec(j))
        fprintf("RbLeg thetas:\n")
        moveCM(xcm(j), ycm(j), zcm(j), footXYZ_W3, 3, A_R_0_rb, gammaVec(j))
        fprintf("LbLeg thetas:\n")
        moveCM(xcm(j), ycm(j), zcm(j), footXYZ_W4, 4, A_R_0_lb, gammaVec(j))
        fprintf("\n")
        COMinside = plotPolygon(footXYZ_W1, footXYZ_W2, footXYZ_W3, footXYZ_W4, Robot_CM_W, safetyFactor, 4);
        view(3)
        grid minor
        pause(0.001)
        hold off
        
    end
     staticPosLeg1 = (Ainv(A_R_W * A_R_0_lf)*footXYZ_W1)';
     staticPosLeg2 = (Ainv(A_R_W * A_R_0_rf)*footXYZ_W2)';
     staticPosLeg3 = (Ainv(A_R_W * A_R_0_rb)*footXYZ_W3)';
     staticPosLeg4 = (Ainv(A_R_W * A_R_0_lb)*footXYZ_W4)';
end

%% forward kinematics
function [A_0_1, A_1_2, A_2_3] = getForwardKinematics(theta1, theta2, theta3)
    global l1 l2 l3 l4
    % D.H parameters

    alpha1 = pi/2; alpha2 = 0 ; alpha3 = 0; a1 = 0; a2 = l3; a3 = l4; d1 = l1; d2 = l2; d3 = 0;

    A_0_1 = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1)
              sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1)
                0 sin(alpha1) cos(alpha1) d1
                    0 0 0 1];
    A_1_2 = [cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2)
              sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2)
                0 sin(alpha2) cos(alpha2) d2
                    0 0 0 1];
    A_2_3 = [cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3)
              sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3)
                0 sin(alpha3) cos(alpha3) d3
                    0 0 0 1];
                
end


%% inverse kinematics option 2
function [theta1, theta2, theta3] = getInverseKinematics(px, py, pz, leg)
    global l1 l2 l3 l4
    switch leg 
        case 1 % lf
            ARM = 1; ELB = 1; aux = -1;
            C1 = -(l2*py+px*sqrt(-l2^2+px^2+py^2))/(px^2+py^2); %no need
            S1 = ARM*sqrt(1-C1^2);% no need
            fi = pi;
        case 2 % rf
            ARM = 1; ELB = 1; aux = 1;
            C1 = -(l2*py-px*sqrt(-l2^2+px^2+py^2))/(px^2+py^2);
            S1 = ARM*sqrt(1-C1^2);
            fi = 0;
        case 3 % rb
            ARM = 1; ELB = -1; aux = -1;
            C1 = -(l2*py+px*sqrt(-l2^2+px^2+py^2))/(px^2+py^2); %no need
            S1 = ARM*sqrt(1-C1^2);% no need
            fi = pi;
        case 4 % lb
            ARM = 1; ELB = -1; aux = 1;
            C1 = -(l2*py-px*sqrt(-l2^2+px^2+py^2))/(px^2+py^2);
            S1 = ARM*sqrt(1-C1^2);
            fi = 0;
    end
    
    temp = ((px^2+py^2+(pz-l1)^2))-l2^2-l3^2-l4^2;
    C3 = temp/(2*l3*l4); S3 = ELB*sqrt(1-C3^2);
    theta3 = aux*atan2(S3,C3);

    % phi = (pi/2)*(ARM*sign(px)-1)
    theta1 = aux * atan2(S1,C1);

    py_new = py+l2*cos(theta1);
    px_new = px-l2*sin(theta1);
    theta2 = fi+aux*(atan2((pz-l1),sqrt(px_new^2+py_new^2))-atan2(l4*S3,(l3+l4*C3)));
end
%%
% this function calculates a vector from the shoulder to the leg end in world coordinates according to desired CM location  
% gamma is is the yao angle (in case the the robot is turning)
%footX/Y/Z are the coordinated of the foot relative to the world
function S_shoulder = getS(cmX_w, cmY_w, cmZ_w, footX_W, footY_W, footZ_W ,leg, A_leg_Rcm, gamma) %S is the vector from the shoulder to the end of the leg relative to the sholder
    global H w AuxVec
    % define bCM ---------------------- change it if CM changes because of mechanicl design ------------------------- 
    switch leg
        case 1 
            b_CM = [-w/2, H/2, 0] - AuxVec; %bCM is a vector from the robot CM to the leg sholder   
        case 2     
            b_CM = [w/2, H/2, 0] - AuxVec;
        case 3
            b_CM = [w/2, -H/2, 0] - AuxVec;
        case 4
            b_CM = [-w/2, -H/2, 0] - AuxVec;
    end
    b_CM = [b_CM 1];
    A_R_W = getRobotToWorldTrans(gamma, cmX_w, cmY_w, cmZ_w); %transform matrix from R to world
    b_W = A_R_W * b_CM'; %b_W is the vector b in world coordinates
    v_foot_w = [footX_W, footY_W, footZ_W, 1];
    S = v_foot_w' - b_W;
    S(4) = 1; 
    A_leg_W = Ainv( A_R_W*A_leg_Rcm); % calc world to leg transform
    S_shoulder = A_leg_W(1:3,1:3) * S(1:3); % calc S vector relative to the leg.
    
%     S_shoulder = A_R_W' * S_shoulder;
end

%% 
function [xVec, yVec, zVec, gammaVec] = CMpathPlan(cmX_init, cmY_init, cmZ_init, gamma_init, cmX_des, cmY_des, cmZ_des, gamma_des) %only forward and backwards for now
    stepsNum = 10;
    xVec = linspace(cmX_init,cmX_des, stepsNum);
    yVec = linspace(cmY_init,cmY_des, stepsNum);
    zVec = linspace(cmZ_init,cmZ_des, stepsNum);
    gammaVec = linspace(gamma_init, gamma_des, stepsNum);
end
%%
function moveCM(cmX_des, cmY_des, cmZ_des, footXYZ_W, leg, A_R_0, gamma)
    A_R_W = getRobotToWorldTrans(gamma, cmX_des, cmY_des, cmZ_des);
    S = getS(cmX_des, cmY_des, cmZ_des, footXYZ_W(1), footXYZ_W(2), footXYZ_W(3) ,leg, A_R_0, gamma); 
    plot_mat = fkin(S(1),S(2),S(3), leg, A_R_0, A_R_W,1);
    robotPlot(plot_mat)
end

%% this functions calculates a 5 degree polynom coefs
function sol = getCoefs(posCond, velCond, accelCond)
     syms a b c d e f t

    % a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t +f
    % 5*a*t^4 + 4*b*t^3 + 3*c*t^2 + 2*d*t + e 
    % 20*a*t^3 + 12*b*t^2 + 6*c*t +2*d

    t0 = posCond(1, 1);
    tf = posCond(2, 1);
    x0 = posCond(1, 2);
    xf = posCond(2, 2);
    v0 = velCond(1, 2);
    vf = velCond(2, 2);
    a0 = accelCond(1, 2);
    af = accelCond(2, 2);


    eqn1 = x0 == a*t0^5 + b*t0^4 + c*t0^3 + d*t0^2 + e*t0 + f;
    eqn2 = xf == a*tf^5 + b*tf^4 + c*tf^3 + d*tf^2 + e*tf + f;
    eqn3 = v0 == 5*a*t0^4 + 4*b*t0^3 + 3*c*t0^2 + 2*d*t0 + e;
    eqn4 = vf == 5*a*tf^4 + 4*b*tf^3 + 3*c*tf^2 + 2*d*tf + e;
    eqn5 = a0 == 20*a*t0^3 + 12*b*t0^2 + 6*c*t0 +2*d;
    eqn6 = af == 20*a*tf^3 + 12*b*tf^2 + 6*c*tf +2*d;

    eqns = [eqn1 eqn2 eqn3 eqn4 eqn5 eqn6];
    vars = [a b c d e f];

    [a, b, c, d, e, f] = solve(eqns, vars);
    sol = [a b c d e f];
end


%% parabolic output curve create func
function [inVecSol, outVecSol] = buildOutputVector(polyCoefs, ti, tf, vin_0, vin_f ,vOut_0, vOut_f, vOut_max)   
    % firstwe need to find the coefs of the output vector
    syms a b c
    vin_max = (vin_f + vin_0)/2;
    
    eqn1 = vOut_0 == a*vin_0^2 + b*vin_0 + c;
    eqn2 = vOut_f == a*vin_f^2 + b*vin_f + c;
    eqn3 = vOut_max == a*vin_max^2 + b*vin_max + c;
    eqns = [eqn1, eqn2, eqn3];
    vars = [a, b, c];
    [a, b, c] = solve(eqns, vars);
    outVecCoefs = [a, b, c];
    % now we build the input Vector and output Vector
    inVec = zeros(1, length(ti:0.01:tf));
    outVec = zeros(1, length(ti:0.01:tf));
    i = 1;
    for t = ti:0.01:tf
        inVec(i) = polyCoefs(1)*t^5 + polyCoefs(2)*t^4 + polyCoefs(3)*t^3 + polyCoefs(4)*t^2 + polyCoefs(5)*t + polyCoefs(6);
        outVec(i) = outVecCoefs(1)*inVec(i)^2 + outVecCoefs(2)*inVec(i) + outVecCoefs(3);
        i = i + 1;
    end
    
    inVecSol = inVec;
    outVecSol = outVec;
end

%%
function plot_mat = fkin(x,y,z,leg,A_R_0,A_R_W, print)
    global l1 l2
    % this function get wanted x.y.z for each joint relative to the base of the leg
    % calculate the inverse kinematics and return the plting matrix
    [theta1, theta2,theta3] = getInverseKinematics(x,y,z,leg);
    
    if print == 1
    fprintf('%f\n', theta1)
    fprintf('%f\n', theta2)
    fprintf('%f\n', theta3)
    end
    
    o = [0 ;0 ;0 ; 1]; % origin
    R = [0;0;0;1]; % joint place vector
    jnt1 = [0 0 l2 1]; % relative to the A_0_1 - for ploting the knee
    knee = [0 0 l1 1]; % relative to the foot - for ploting the knee

            
    [A_0_1, A_1_2, A_2_3] = getForwardKinematics(theta1, theta2, theta3);
    jnt1 = double(A_0_1 * jnt1') ; jnt2 = double(A_0_1 * A_1_2 * o) ; jnt3 = double(A_0_1 * A_1_2 * A_2_3 * o);
    o_cm = A_R_0*o ; knee_cm = A_R_0*knee' ; jnt1_cm=A_R_0*jnt1 ; jnt2_cm=A_R_0*jnt2 ; jnt3_cm=A_R_0*jnt3;
    o_W=A_R_W*o_cm; knee_W = A_R_W*knee_cm ;jnt1_W=A_R_W*jnt1_cm ; jnt2_W=A_R_W*jnt2_cm ; jnt3_W=A_R_W*jnt3_cm;
    plot_mat = [o_W'; knee_W'; jnt1_W'; jnt2_W'; jnt3_W'];

end

%%
function [x, y] = getXYZforSpin(gama, xi, zi, leg) % x, y defined in the world axis and xi, zi are relative to the leg   
    global H w
    length = H + 2*abs(zi);
    width = w + 2*abs(xi);
    if leg == 1 || leg == 3
        b = atan2(width/2,length/2);
        C = (pi-gama)/2;
        a = C-b;
        Z = sqrt(length^2+width^2)*sin(gama/2);
        y = Z*cos(a) ;
        x = Z*sin(a)+xi;

    else % if leg 2 or 4
        b = atan2(length/2,width/2);
        C = (pi-gama)/2;
        a = C-b;
        Z = sqrt(length^2+width^2)*sin(gama/2);
        x = Z*cos(a)-xi; % lengthowidth muclength to spin in x 
        y = Z*sin(a);
    end
end

