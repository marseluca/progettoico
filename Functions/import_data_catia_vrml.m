function map = import_data_catia_vrml(filename,scalefactor)

% ATTENTION!  This is a simple code to extract points and lines from VRML
% exported by CATIA!

% READ_VRML imports VRML data saves in "filename" to a Matlab variable "vrml_struct". 
% This approach assumes that it is possible to read the entire file to
% memory

% open the file
[fid, message] = fopen(filename,'r');
if fid<0
    % disp(message)
    return
end

% Read the entire file
[vrml_file,vrml_file_length] = fread(fid); %#ok<NASGU>

% Close file
fclose(fid);

% For convenience
vrml_file = vrml_file';

% initialization
map.points = [];
map.lines = [];
% initialization of rot and translation matrices
rot = eye(3);
trans = zeros(3,1);
scale = ones(3,1);

% ignore the PROTO


% find and replace all DEF/USE by numbers for direct index
% disp('Replacing strings...')
[definitions,vrml_file] = findReplaceDEF_USE(vrml_file);

% The code was initial optimized without words DEF and USE in the VRML
% file. To continue using this code, "USE" is replaced by "U{}"
vrml_file = double(regexprep(char(vrml_file),'USE','U{}'));

% find all open brackets
ind_open_brackets = findOpenBrackets(vrml_file);
% find all close brackets
ind_close_brackets = findCloseBrackets(vrml_file,ind_open_brackets);
% all brackets sorted
ind_previous_near_bracket = findPreviousNearBracket(ind_open_brackets,ind_close_brackets);

% start % complete
% disp('Parsing VRML...')
nReady(0)
% Find points and line recursively
[map.points,map.lines,definitions,file_complete] = findPointsLines(...
                                 map.points,...
                                 map.lines,...
                                 vrml_file,...
                                 scale,...
                                 rot,...
                                 trans,...
                                 ind_previous_near_bracket,...
                                 ind_open_brackets,...
                                 ind_close_brackets,...
                                 definitions,0);       %#ok<NASGU,NASGU>
% finish % complete
nReady(100)

% to follow the map format
map.lines = sort(map.lines,1);


% Plotting data
disp('Drawing...')
% Plot points
plot3(map.points(1,:)*scalefactor,map.points(2,:)*scalefactor,map.points(3,:)*scalefactor,'ob')
cla
hold on
% plot the lines
for f=1:size(map.lines,2)
    plot3([map.points(1,map.lines(1,f))*scalefactor map.points(1,map.lines(2,f))*scalefactor],...
          [map.points(2,map.lines(1,f))*scalefactor map.points(2,map.lines(2,f))*scalefactor],...
          [map.points(3,map.lines(1,f))*scalefactor map.points(3,map.lines(2,f))*scalefactor],...
          'r','LineWidth',1.5);
end
hold off
xlabel('X-axis');  % Label for the X-axis
ylabel('Y-axis');  % Label for the Y-axis
zlabel('Z-axis');  % Label for the Z-axis
daspect([1 1 1])


% extract a 2D map
% map = extract2Dmap(map);

% save points 
% uisave('map','map*.mat')



%% Find points and line recursively
function [points,lines,definitions,file_complete] = findPointsLines(points,lines,str,scale,rot,trans,ind_previous_near_bracket,ind_open_brackets,ind_close_brackets,definitions,file_complete)

% to initialize the variables where the transformation parameters will be
% saved
old_rot = rot;
old_trans = trans;
old_scale = scale;

%     Example
%     s =
%     a{b{c}d {e f} g{h{i}j}k}l{m}
%     ind_open_brackets =
%          2     4     9    16    18    26
%     ind_close_brackets =
%         24     6    13    22    20    28

% init ind
ind1 = 1;
% if at least there is a bracket
while ind1<=length(ind_open_brackets)
    % complete
    file_complete = max(file_complete,ind_open_brackets(ind1)/length(str));
    
    % Find the previous word
	[word,ind_word] = findWord(str(ind_previous_near_bracket(ind1):ind_open_brackets(ind1)-1));
    
    % VRML syntax
    switch char(word)

        % Transformation
        case {'Transform'}
            % to save the previous transformation
            old_rot = rot;
            old_trans = trans;
            old_scale = scale;

            new_rot = getNewRot(str(ind_open_brackets(ind1):ind_open_brackets(ind1+1)));
            trans = rot * getNewTrans(str(ind_open_brackets(ind1):ind_open_brackets(ind1+1))) + trans;
            rot = rot * new_rot;
            scale = getNewScale(str(ind_open_brackets(ind1):ind_open_brackets(ind1+1))) .* scale;

        % IndexedLineSet
        case {'IndexedLineSet'}
            % number of points
            n_points = size(points,2);
            % get points
            new_points = getNewPoints(str(ind_open_brackets(ind1):ind_close_brackets(ind1)));
            new_points = transformPoints(new_points,scale,rot,trans);
            points = [points new_points];
            % get lines
            new_lines = getNewLines(str(ind_open_brackets(ind1):ind_close_brackets(ind1)))+n_points;
            lines = [lines new_lines]; %#ok<AGROW>
            % break the while
            break

        % set pre-defined commands
        case {'DEF'}
            % find the respective set of points
            ind2 = find(ind_open_brackets(ind1+1:end) < ind_close_brackets(ind1))+ind1;
            ind2 = max([ind1 ind2]);
            
            % identify the definition (30 is only for optimization)
            def = strtok(str(ind_previous_near_bracket(ind1)+ind_word+2:ind_previous_near_bracket(ind1)+ind_word+30));
            % find the respective set of points
            ind = findDEF(def);

            % add a new definition
            definitions(ind).ind_open_brackets = ind_open_brackets(ind1:ind2);
            definitions(ind).ind_close_brackets = ind_close_brackets(ind1:ind2);
            definitions(ind).ind_previous_near_bracket = ind_previous_near_bracket(ind1:ind2);

        % use pre-defined commands
        case {'U'}
            % identify the definition (30 is only for optimization)
            def = strtok(str(ind_open_brackets(ind1)+3:ind_open_brackets(ind1)+30));
            % find the respective set of points
            ind = findDEF(def);
            [points,lines,definitions,file_complete_new] = findPointsLines(...
                            points,...
                            lines,...
                            str,...
                            scale,...
                            rot,...
                            trans,...
                            definitions(ind).ind_previous_near_bracket,...
                            definitions(ind).ind_open_brackets,...
                            definitions(ind).ind_close_brackets,...
                            definitions,...
                            file_complete);
            
            % complete
            file_complete = max(file_complete,file_complete_new);

        % by default
        otherwise
    end
    % if there is another brackets inside
    ind2 = find(ind_open_brackets(ind1+1:end) < ind_close_brackets(ind1))+ind1;

    if ~isempty(ind2)
        ind2 = max(ind2);
        ind1 = ind1+1;
           % A TEST FOR CORRUPTED OR WRONG VRML CODE MUST BE HERE
        local_ind_open_brackets = ind_open_brackets(ind1:ind2);
        local_ind_close_brackets = ind_close_brackets(ind1:ind2);
        [points,lines,definitions,file_complete_new] = findPointsLines(points,lines,str,scale,rot,trans,...
            ind_previous_near_bracket(ind1:ind2),local_ind_open_brackets,local_ind_close_brackets,definitions,file_complete);
        ind1 = ind2;

        % to return to the previous transformation
        rot = old_rot;
        trans = old_trans;
        scale = old_scale;
        % complete
        file_complete = max(file_complete,file_complete_new);

    else
        % complete
        nReady(100*file_complete)  
    end


    % next command
    ind1 = ind1+1;
end


%% Find the last word before "{"
function [word,ind] = findWord(str)

% Find DEF and USE
ind = findstr(str,'DEF');
if ind
    word = 'DEF';
else
    % Find USE
    ind = findstr(str,'USE');
    if ind
        word = 'USE';
    else
        word = fliplr(strtok(fliplr(str)));
    end
end


        
%% Get the new rotation matrix
function scale = getNewScale(str)

% small caps
scale_str = lower('scale');

ind = findstr(lower(str),scale_str)+length(scale_str);
eval_tmp = eval(['[' strtok(str(ind:end),10) ']']);
if isempty(eval_tmp)
    scale = [1 1 1]';
else  
    scale = eval_tmp(1:3)';
end


%% Get the new translation vector
function trans = getNewTrans(str)

% small caps
translation_str = lower('translation');

ind = findstr(lower(str),translation_str)+length(translation_str);
eval_tmp = eval(['[' strtok(str(ind:end),10) ']']);
if isempty(eval_tmp)
    trans = [0 0 0]';
else
    trans = eval_tmp(1:3)';
end


%% Get the new scale vector
function rot = getNewRot(str)

% small caps
rotation_str = 'rotation';

ind = findstr(lower(str),rotation_str)+length(rotation_str);
eval_tmp = eval(['[' strtok(str(ind:end),10) ']']);
if isempty(eval_tmp)
    rot = eye(3);
else
    v = eval_tmp(1:3)';
    angle = eval_tmp(end);
    rot = getRotationMat(v,angle);
end

%% Get the new points
function new_points = getNewPoints(str)

% find points
points_str = 'point';

ind1 = findstr(lower(str),points_str);
% find [
ind_open = find(str(ind1+length(points_str):end)==91,1)+ind1+length(points_str)-1;
% find ]
ind_close = find(str(ind1+length(points_str):end)==93,1)+ind1+length(points_str);
% evaluate the truncated string
new_points = eval(char(str(ind_open:ind_close)))';


%% Get the new lines
function new_lines = getNewLines(str)

% find coordIndex
lines_str = 'coordIndex';

ind1 = findstr(lower(str),lines_str);
% find [
ind_open = find(str(ind1+length(lines_str):end)==91,1)+ind1+length(lines_str)-1;
% find ]
ind_close = find(str(ind1+length(lines_str):end)==93,1)+ind1+length(lines_str);
% evaluate the truncated string
new_lines = findLines(eval(char(str(ind_open:ind_close))))';


%% Find Lines from coordIndex
function lines = findLines(coordIndex)

% +1 since in VRML indices start in 0
coordIndex = coordIndex + 1;

lines = [];
for f=1:length(coordIndex)-1
    if coordIndex(f)>0 && coordIndex(f+1)>0
        lines = [lines; coordIndex(f) coordIndex(f+1)]; %#ok<AGROW>
    end
end


%% Rotation of points (Quaternion matrix -> Rotation Matrix)
function rot = getRotationMat(v,angle)

% based on Quaternions:
% http://en.wikipedia.org/wiki/Rotation_matrix#Axis_and_angle

c = cos(angle); s = sin(angle); C = 1-c;

xs = v(1)*s;   ys = v(2)*s;   zs = v(3)*s;
xC = v(1)*C;   yC = v(2)*C;   zC = v(3)*C;
xyC = v(1)*yC; yzC = v(2)*zC; zxC = v(3)*xC;

rot = [ v(1)*xC+c   xyC-zs      zxC+ys ; ...
        xyC+zs      v(2)*yC+c   yzC-xs ; ...
        zxC-ys      yzC+xs      v(3)*zC+c ];


%% Translation of points
function p = translatePoints(p,t)

p(1,:) = p(1,:) + t(1);
p(2,:) = p(2,:) + t(2);
p(3,:) = p(3,:) + t(3);


%% Scale points
function p = scalePoints(p,s)

p(1,:) = p(1,:) * s(1);
p(2,:) = p(2,:) * s(2);
p(3,:) = p(3,:) * s(3);


%% Scale points
function p = rotatePoints(p,rot)

p = rot*p;


%% Transform points
function p = transformPoints(p,s,rot,t)

% rotation
p = rotatePoints(p,rot);
% translation
p = translatePoints(p,t);
% scale
p = scalePoints(p,s);


%% Find open Brackets "{"
function ind = findOpenBrackets(s)

ind = find(s==123); % {


%% Find close Brackets "}"
function ind_close_brackets = findCloseBrackets(s,ind_open_brackets)

% init of ind_close_brackets
ind = (s==125); % }
brackets(ind_open_brackets) = 1;
brackets(ind) = -1;

tic

% init
ind_close_brackets = zeros(size(ind_open_brackets));
ind1 = [];
ind2 = 0;
for f=1:length(brackets)
    if brackets(f)>0
        ind2 = ind2 + 1;
        ind1(end+1) = ind2;
    elseif brackets(f)<0
        ind_close_brackets(ind1(end)) = f;
        ind1(end) = [];
    end
end


%% To show how much is ready
function nReady(n)

switch n
    case 0 % start
        fprintf('    ')
    case 100 % stop
        fprintf('\b\b\b\b\b Done.\n')
    otherwise % value
        fprintf('\b\b\b\b%3.0f%%',n)
end


%% Find the associated definition
function ind = findDEF(def)

ind = str2double(char(def));


%% Find Previous Near Bracket
function ind_previous_near_bracket = findPreviousNearBracket(ind_open_brackets,ind_close_brackets)

[ind_all_brackets ind_ind_sorted] = sort([ind_open_brackets ind_close_brackets]);
ind_open_brackets_sorted = find(ind_ind_sorted<=length(ind_open_brackets));

ind_previous_near_bracket = ones(1,length(ind_open_brackets));
for f=2:length(ind_open_brackets)
    ind_previous_near_bracket(f) = ind_all_brackets(ind_open_brackets_sorted(f)-1)+1;
end


%% Find and replace all DEF/USE by numbers for direct index
function [definitions,str] = findReplaceDEF_USE(str)

% definitions of VRML
definitions = [];

str_new = char(str);
ind_DEF = findstr(str,'DEF ');
for f=1:length(ind_DEF)
    word = char(strtok(str(ind_DEF(f)+4:ind_DEF(f)+30)));
    % disp(word)
    definitions(end+1).id = word;
    str_new = regexprep(str_new,word,num2str(f));
end
str = double(str_new);
