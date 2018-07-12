function [fitresult, gof] = createFit(x, y, Map1)
%CREATEFIT(X,Y,MAP1)
%  Create a fit.
%
%  Data for 'gas distribution fitting' fit:
%      X Input : x
%      Y Input : y
%      Z Output: Map1
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 27-Jul-2017 10:13:59


%% Fit: 'gas distribution fitting'.
[xData, yData, zData] = prepareSurfaceData( x, y, Map1 );

% Set up fittype and options.
ft = fittype( 'exp(-(x-a)^2/(2*c*c)-(y-b)^2/(2*d*d))', 'independent', {'x', 'y'}, 'dependent', 'z' );
excludedPoints = (zData < 0) | (zData > 1);
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [-30 -30 15 30];
opts.Exclude = excludedPoints;

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, opts );

% Plot fit with data.
figure( 'Name', 'gas distribution fitting' );
h = plot( fitresult, [xData, yData], zData, 'Exclude', excludedPoints );
legend( h, 'gas distribution fitting', 'Map1 vs. x, y', 'Excluded Map1 vs. x, y', 'Location', 'NorthEast' );
% Label axes
xlabel x
ylabel y
zlabel Map1
grid on

