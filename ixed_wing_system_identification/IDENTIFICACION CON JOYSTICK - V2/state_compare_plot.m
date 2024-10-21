% Create a figure with 2 rows and 3 columns
figure;

% Load data
load fullStateData_v3_zeroNoise.mat
load validationStateData.mat

% Data for the plots (you can replace this with your own data)
t = time;
y1_1 = x;       % First line data
y1_2 = xVAL;    % Second line data

y2_1 = y;       % First line data
y2_2 = yVAL;    % Second line data

y3_1 = z;       % First line data
y3_2 = zVAL;    % Second line data

y4_1 = phi;     % First line data
y4_2 = phiVAL;  % Second line data

y5_1 = theta;   % First line data
y5_2 = thetaVAL;% Second line data

y6_1 = psi;     % First line data
y6_2 = psiVAL;  % Second line data

% Set font name
fontName = 'Times New Roman';

% Turn off LaTeX interpreter
set(groot, 'defaultAxesTickLabelInterpreter','none');
set(groot, 'defaultLegendInterpreter','none');

% Set line thickness
lineThickness = 2;

% First subplot
subplot(2, 3, 1);
plot(t, y1_1, 'r', t, y1_2, '--b', 'LineWidth', lineThickness);
xlabel('X-axis Label', 'FontName', fontName);
ylabel('Y-axis Label', 'FontName', fontName);
grid minor;

% Second subplot
subplot(2, 3, 2);
plot(t, y2_1, 'r', t, y2_2, '--b', 'LineWidth', lineThickness);
xlabel('X-axis Label', 'FontName', fontName);
ylabel('Y-axis Label', 'FontName', fontName);
grid minor;

% Third subplot
subplot(2, 3, 3);
plot(t, y3_1, 'r', t, y3_2, '--b', 'LineWidth', lineThickness);
xlabel('X-axis Label', 'FontName', fontName);
ylabel('Y-axis Label', 'FontName', fontName);
grid minor;

% Fourth subplot
subplot(2, 3, 4);
plot(t, y4_1, 'r', t, y4_2, '--b', 'LineWidth', lineThickness);
xlabel('X-axis Label', 'FontName', fontName);
ylabel('Y-axis Label', 'FontName', fontName);
grid minor;

% Fifth subplot
subplot(2, 3, 5);
plot(t, y5_1, 'r', t, y5_2, '--b', 'LineWidth', lineThickness);
xlabel('X-axis Label', 'FontName', fontName);
ylabel('Y-axis Label', 'FontName', fontName);
grid minor;

% Sixth subplot
subplot(2, 3, 6);
plot(t, y6_1, 'r', t, y6_2, '--b', 'LineWidth', lineThickness);
xlabel('X-axis Label', 'FontName', fontName);
ylabel('Y-axis Label', 'FontName', fontName);
grid minor;

% Adjust the layout of the plots
sgtitle('Six Plots in One Figure', 'FontName', fontName);
