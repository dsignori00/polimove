function path = get_tracking_dir()
    proj = currentProject();

    if strcmp(proj.Name, 'polimove')
        path = fullfile(proj.RootFolder, 'src', 'perception', 'tracking');
    else
        error('Unknown project: %s', proj.Name);
    end
end