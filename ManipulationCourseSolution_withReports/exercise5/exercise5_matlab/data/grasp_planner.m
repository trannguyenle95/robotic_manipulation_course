% ELEC-E8126 Exercise 5
% Optimal grasp planner
% Alexander Järvistö
% 529057

clear all;
close all;
clc;

%% Load data
vertices = load('vertices.txt');
plotMargin = 2; % Can be adjusted

%% Plot data
hold on;
lines = [];
for i = 1:length(vertices(:,1))
    plot(vertices(i,1), vertices(i,2), '.b', 'MarkerSize', 10);
end
for i = 2:length(vertices(:,1))
    lines = [lines; plot([vertices(i-1,1) vertices(i,1)], [vertices(i-1,2) vertices(i,2)], 'blue')];
end
lines = [lines; plot([vertices(1,1) vertices(i,1)], [vertices(1,2) vertices(i,2)], 'blue')];
axis equal 
ylim([-plotMargin plotMargin]);
xlim([-plotMargin plotMargin]);
Pm = [mean(vertices(:,1)) mean(vertices(:,2))];

%% Find grasp
isDone = 0;
offset = 0;
bestA = 99999;

% Simple start
for i = 1:length(lines)
    Pa = getLineMean(lines(i));
    [Pb, pbLine] = pointOnOtherSide(Pa, lines(i), lines, offset); % Get the point that is accross the object
    if ~isnan(Pb)
        k = (Pb(2)-Pa(2))./(Pb(1)-Pa(1));
        a1 = getAngleK2Norm(k,lines(i)); % Calculate the angle between the line and surface normal
        a2 = getAngleK2Norm(k,pbLine);
        atot = a1+a2;
        if abs(atot) < 0.001 % Is this a perfect grasp?
            isDone = 1;
            bestPa = Pa;
            bestPb = Pb;
            bestPaLine = lines(i);
            bestPbLine = pbLine;
            bestA = atot;
            break
        end
    end
end



% Do it the hard way then
for lineNum = 1:length(lines) % Test every line
    if isDone
        break
    end
    [xv, yv] = getLinePoints(lines(lineNum),20); % Get points on line to test
    for pointNum = 1:length(xv)
        if isDone
            break
        end
        Pa(1) = xv(pointNum);
        Pa(2) = yv(pointNum);
        if isLineCorner(Pa, lines(lineNum))
            continue % Skip corners
        end
        for angleOffset = -45:2:45 % Test different angles
            [Pb, pbLine] = pointOnOtherSide(Pa, lines(lineNum), lines, angleOffset);
            if ~isnan(Pb)
                k = (Pb(2)-Pa(2))./(Pb(1)-Pa(1));
                a1 = getAngleK2Norm(k,lines(lineNum));
                a2 = getAngleK2Norm(k,pbLine);
                if a1 > 45 || a2 > 45
                    continue % must be inside friction cone
                end
                atot = a1 + a2;
                if abs(atot) < 0.001 % Found a perfect one
                    bestPa = Pa;
                    bestPb = Pb;
                    bestPaLine = lines(lineNum);
                    bestPbLine = pbLine;
                    bestA = atot;
                    break
                elseif atot < bestA % Found a better one
                    bestPa = Pa;
                    bestPb = Pb;
                    bestPaLine = lines(lineNum);
                    bestPbLine = pbLine;
                    bestA = atot;
                end
            end
        end
   end
end

% Save the best grasp found
Pa = bestPa;
Pb = bestPb;
PaLine = bestPaLine;
PbLine = bestPbLine;

% Should not happen
if isnan(sum(Pa)) || isnan(sum(Pb))
    fprintf('Could not find grasp! Sorry!\n');
    text(Pm(1),Pm(2), 'FAILED TO FIND GRASP', 'Color', 'red', 'FontSize', 14, 'HorizontalAlignment', 'center');
    return;
end

%% Plot found grasp
plotCone(Pa, PaLine, lines, 1/255*[128,0,0]);
plotCone(Pb, PbLine, lines, 1/255*[0,100,0]);
daLine = line([Pa(1), Pb(1)],[Pa(2), Pb(2)], 'LineStyle', '--', 'Color', 1/255*[255,140,0]);
Pap = plot(Pa(1), Pa(2), '.r', 'MarkerSize', 20);
Pbp = plot(Pb(1), Pb(2), '.g', 'MarkerSize', 20);
expandLine(daLine, 'red', 'green');
legend([Pap Pbp], 'Grasp point 1', 'Grasp point 2')

%% Process grasp

% Make sure left and right points have correct angles
if Pa(1) < 0
    pointL = Pa*0.1;
    pointR = Pb*0.1;
    kf = getLineCoeff(daLine);
    if isinf(kf)
        if kf < 0
            angleL = 270;
            angleR = 90;
        else
            angleL = 90;
            angleR = 270;
        end
    elseif abs(kf) < 0.001
        angleL = 0;
        angleR = 180;
    else
        angle = atand(kf);
        angleR = 180 + angle;
        angleL = angleR + 180;
        if angleL > 360
            angleL = angleL - 360;
        end
       
    end
else
    pointR = Pa*0.1;
    pointL = Pb*0.1;
    kf = getLineCoeff(daLine);
    if isinf(kf)
        if kf < 0
            angleR = 270;
            angleL = 90;
        else
            angleR = 90;
            angleL = 270;
        end
    elseif abs(kf) < 0.001
        angleL = 0;
        angleR = 180;
    else
        angle = atand(kf);
        angleR = 180 + angle;
        angleL = angleR + 180;
        if angleL > 360
            angleL = angleL - 360;
        end
    end
end


%% Save grasp to file
%Right 1st line
%Left: 2nd line
fileID = fopen('grasp_data.txt','w');
fprintf(fileID, '%.4f %.4f %.4f\n', pointR, angleR);
fprintf(fileID, '%.4f %.4f %.4f\n', pointL, angleL);
fclose(fileID);

%% Print results
fprintf('Found optimal grasp:\n');
fprintf('Grasp point right: %.4f %.4f %.4f\n', pointR, angleR);
fprintf('Grasp point left: %.4f %.4f %.4f\n', pointL, angleL);
if bestA < 0.001
    Q = 100;
else
    Q = (1-bestA/180)*100;
end
fprintf('Grasp quality: %.2f%%\n', Q);

%% Functions

% Checks if point P is an endpoint of line
function a = isLineCorner(P, line)
    x1 = line.XData(1);
    x2 = line.XData(2);
    y1 = line.YData(1);
    y2 = line.YData(2);
    if (abs(P(1)-x1) < 0.0001) && (abs(P(2)-y1) < 0.0001)
        a = 1;
    elseif (abs(P(1)-x2) < 0.0001) && (abs(P(2)-y2) < 0.0001)
        a = 1;
    else
        a = 0;
    end
end

% Expand the grasp point line outside the object
function expandLine(line, color1, color2)
    d = 1.0; % length of line (w.r.t half of line)
    
    x1 = line.XData(1);
    x2 = line.XData(2);
    y1 = line.YData(1);
    y2 = line.YData(2);
    xm = mean(line.XData);
    ym = mean(line.YData);
    
    Pa = [x1 y1];
    Pb = [x2 y2];
    Pm = [xm ym];
    
    Va = Pa + (Pa - Pm)*d;
    Vb = Pb + (Pb - Pm)*d;
    plot([Pa(1), Va(1)], [Pa(2), Va(2)], color1);
    plot([Pb(1), Vb(1)], [Pb(2), Vb(2)], color2);   
end

% Get the angle between two lines
function angle = getAngleK2Norm(k1,base)
    k2 = getLineCoeff(base);
    k2 = -1./k2;    
    if isinf(k1) && isinf(k2)
        angle = 0;
    elseif isinf(k2)
        angle = 90 - atand(k1);
    elseif isinf(k1)
        angle = 90 - atand(k2);
    else
       angle = atand(abs((k1-k2)./(1+k1*k2)));
        if abs(angle) < 0.001
            angle = 0;
        end 
    end
    if isnan(angle)
        angle = 0;
    end
end

% Find the point on the other side with a offset
function [point, line] = pointOnOtherSide(A, line, lines, offset)
    tol = 1e-8;
    k = getLineCoeff(line);
    k = -1./k;
    x = A(1);
    y = A(2);
    point(1) = NaN;
    point(2) = NaN;
    line = NaN;
    for i = 1:length(lines)
        [hx, hy] = getHitPoint(atand(k)+ offset, x, y, lines(i));
        if (abs(hx-x) < tol) && (abs(hy-y) < tol)
            continue; % Same as origin
        end
        x1 = lines(i).XData(1);
        x2 = lines(i).XData(2);
        y1 = lines(i).YData(1);
        y2 = lines(i).YData(2);
        if x1 > x2
            xupper = x1;
            xlower = x2;
        else
            xupper = x2;
            xlower = x1;
        end
        if y1 > y2
            yupper = y1;
            ylower = y2;
        else
            yupper = y2;
            ylower = y1;
        end
        % Check if point is on visible line
        if ~(abs(hx-xlower) < tol) && (hx > xupper || hx < xlower)
            continue;
        end
        if ~(abs(hy-ylower) < tol) && (hy > yupper || hy < ylower)
            continue;
        end
        if (abs(hx-x1) < tol) && (abs(hy-y1) < tol)
            continue
        end
        if (abs(hx-x2) < tol) && (abs(hy-y2) < tol)
            continue
        end 
        if ~isnan(hx) && ~isnan(hy)
            point(1) = hx;
            point(2) = hy;
            line = lines(i);
            break;
        end
    end
    
end

% Get line slope
function k = getLineCoeff(line)
    ya = line.YData(1);
    yb = line.YData(2);
    xa = line.XData(1);
    xb = line.XData(2);
    k = (yb-ya)./(xb-xa);
end

% Get line middle point
function point = getLineMean(line)
    point(1) = mean(line.XData);
    point(2) = mean(line.YData);
    if abs(point(1)) < 0.001
        point(1) = 0;
    end
    if abs(point(2)) < 0.001
        point(2) = 0;
    end
end

% Plot the friction cone
function plotCone(P, line, lines, color)
    x = P(1);
    y = P(2);
    tol = 1e-8;
    if abs(x) < 0.001
        x = 0;
    end
    if abs(y) < 0.001
        y = 0;
    end
    slope = getLineCoeff(line);
    if isnan(slope)
        angle = 0;
    else
        angle = atand(slope);
    end
    hitsX = [];
    hitsY = [];
    for i = 1:length(lines) % Find where to end cone line
        [hx, hy] = getHitPoint(angle+45,x,y,lines(i));
        if (abs(hx-x) < tol) && (abs(hy-y) < tol)
            continue;
        end
        x1 = lines(i).XData(1);
        x2 = lines(i).XData(2);
        y1 = lines(i).YData(1);
        y2 = lines(i).YData(2);
        if x1 > x2
            xupper = x1;
            xlower = x2;
        else
            xupper = x2;
            xlower = x1;
        end
        if y1 > y2
            yupper = y1;
            ylower = y2;
        else
            yupper = y2;
            ylower = y1;
        end
        
        if ~(abs(hx-xlower) < tol) && (hx > xupper || hx < xlower)
            continue;
        end
        if ~(abs(hy-ylower) < tol) && (hy > yupper || hy < ylower)
            continue;
        end
        if ~isnan(hx) && ~isnan(hy)
            hitsX = [hitsX; [x ,hx]];
            hitsY = [hitsY; [y ,hy]];
        end
    end
    if length(hitsX) == 2
        plot(hitsX(1,:), hitsY(1,:), 'Color', color);
    else
        firstX = hitsX(1,:);
        firstY = hitsY(1,:);
        firstD = sqrt((hitsX(1,2)-hitsX(1,1)).^2 + (hitsY(1,2)-hitsY(1,1)).^2);
        for i = 2:length(hitsX(:,1))
            d = sqrt((hitsX(i,2)-hitsX(i,1)).^2 + (hitsY(i,2)-hitsY(i,1)).^2);
            if d < firstD
                firstX = hitsX(i,:);
                firstY = hitsY(i,:);
                firstD = d;
            end
        end
        plot(firstX, firstY, 'Color', color); % Only plot to the first line hit
    end
    hitsX = [];
    hitsY = [];
    for i = length(lines):-1:1 % Repeat for other leg
        [hx, hy] = getHitPoint(angle+45+90,x,y,lines(i));
        if (abs(hx-x) < tol) && (abs(hy-y) < tol)
            continue;
        end
        x1 = lines(i).XData(1);
        x2 = lines(i).XData(2);
        y1 = lines(i).YData(1);
        y2 = lines(i).YData(2);
        if x1 > x2
            xupper = x1;
            xlower = x2;
        else
            xupper = x2;
            xlower = x1;
        end
        if y1 > y2
            yupper = y1;
            ylower = y2;
        else
            yupper = y2;
            ylower = y1;
        end
        
        if ~((abs(hx-xupper) < tol) || (abs(hx-xlower) < tol) ) && (hx > xupper || hx < xlower)
            continue;
        end
        if ~((abs(hy-yupper) < tol) || (abs(hy-ylower) < tol) ) && (hy > yupper || hy < ylower)
            continue;
        end
        if ~isnan(hx) && ~isnan(hy)
            hitsX = [hitsX; [x ,hx]];
            hitsY = [hitsY; [y ,hy]];
        end
    end
    if length(hitsX) == 2
        plot(hitsX(1,:), hitsY(1,:), 'Color', color);
    else
        firstX = hitsX(1,:);
        firstY = hitsY(1,:);
        firstD = sqrt((hitsX(1,2)-hitsX(1,1)).^2 + (hitsY(1,2)-hitsY(1,1)).^2);
        for i = 2:length(hitsX(:,1))
            d = sqrt((hitsX(i,2)-hitsX(i,1)).^2 + (hitsY(i,2)-hitsY(i,1)).^2);
            if d < firstD
                firstX = hitsX(i,:);
                firstY = hitsY(i,:);
                firstD = d;
            end
        end
        plot(firstX, firstY, 'Color', color);
    end
end

% Check if and where a line hits another line
function [x, y] = getHitPoint(alpha, x1, y1, line)
    ya = line.YData(1);
    yb = line.YData(2);
    xa = line.XData(1);
    xb = line.XData(2);
    k = (yb-ya)./(xb-xa);
    if isinf(k)
        x = xa;
        y = tand(alpha)*(x-x1)+y1;
    elseif isinf(tand(alpha))
        x = x1;
        y = k*(x-xa)+ya;
    else
        a = (ya-y1+x1*tand(alpha)-k*xa);
        b = (tand(alpha)-k);
        x = a./b;
        y = tand(alpha)*(x-x1)+y1;
    end
    if y > 1000 || y < -1000
        y = NaN;
    end
    if x > 1000 || x < -1000
        x = NaN;
    end
    if abs(x) < 0.001
        x = 0;
    end
    if abs(y) < 0.001
        y = 0;
    end
end

% Split a line into points
function [xv,yv] = getLinePoints(line, numPts)
        x1 = line.XData(1);
        x2 = line.XData(2);
        y1 = line.YData(1);
        y2 = line.YData(2);
        k = getLineCoeff(line);
        if isinf(k) 
            xv(1:numPts)=x1;
            yv(1:numPts)=linspace(y1,y2,numPts);
        elseif abs(k) < 0.0001
            xv(1:numPts) = linspace(x1,x2,numPts);
            yv(1:numPts) = y1;
        else
            xv = linspace(x1,x2,numPts);
            yv = k*(xv-x1) + y1;
        end
        
        % Start from middle
        half = ceil(length(xv)/2);
        A = xv(1:half);
        B = xv(half + 1 : end);
        xv(1:2:2*numel(A)) = A;
        xv(2:2:end) = B;
        
        half = ceil(length(yv)/2);
        A = yv(1:half);
        B = yv(half + 1 : end);
        yv(1:2:2*numel(A)) = A;
        yv(2:2:end) = B;
end
