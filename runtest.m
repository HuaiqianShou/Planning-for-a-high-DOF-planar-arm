function[numofmoves, caught] = runtest(mapfile, armstart, armgoal, planner_id)
LINKLENGTH_CELLS=10;
envmap = load(mapfile);

close all;

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(envmap'); axis square; colorbar; colormap jet; hold on;

%armplan should be a matrix of D by N 
%where D is the number of DOFs in the arm (length of armstart) and
%N is the number of steps in the plan 
armplan = armplanner(envmap, armstart, armgoal, planner_id); 

fprintf(1, 'plan of length %d was found\n', size(armplan,1));
tol = 0.1;
caught=true;


if (norm(armplan(1,:)-armstart,2) > tol)
    fmt=['Error: first conf in armplan =' repmat(' %0.2f',1,numel(armstart)), '\n'];
    fprintf(fmt,armplan(1,:))
    fmt=['but armstart =' repmat(' %0.2f',1,numel(armstart)), '\n'];
    fprintf(fmt,armstart)
    caught = false;
end


if (norm(armplan(end,:)-armgoal,2) > tol)
    fmt=['Error: last conf in armplan =' repmat(' %0.2f',1,numel(armgoal)), '\n'];
    fprintf(fmt,armplan(end,:))
    fmt=['but armgoal =' repmat(' %0.2f',1,numel(armgoal)), '\n'];
    fprintf(fmt,armgoal)
    caught=false;
end

%draw the plan
midx = size(envmap,2)/2;
x = zeros(length(armstart)+1,1);
x(1) = midx;
y = zeros(length(armstart)+1,1);
for i = 1:size(armplan)
    for j = 1:length(armstart)
        x(j+1) = x(j) + LINKLENGTH_CELLS*cos(armplan(i,j));
        y(j+1) = y(j) + LINKLENGTH_CELLS*sin(armplan(i,j));
    end
    plot(x,y, 'c-');
    pause(0.1);
end

%armplan
