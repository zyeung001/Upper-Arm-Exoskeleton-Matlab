function fig = new_fig(sz)
%NEW_FIG Figure window with the repo's export conventions.
%   fig = NEW_FIG([w h]) pops up in a desktop session, stays hidden under
%   matlab -batch, and forces a light theme so exports never follow a dark
%   desktop theme. Shared by every figure producer (make_figures, cl_figures,
%   cl_grid_figure, cl_kappa_figure).
vis = 'off';
if usejava('desktop'), vis = 'on'; end
fig = figure('Visible', vis, 'Color', [252 252 251]/255, 'Position', [80 80 sz]);
fig.Theme = 'light';
end
