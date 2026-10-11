function path = get_gt_dir()
    proj = currentProject();

    if strcmp(proj.Name, 'polimove')
        path = fullfile(proj.RootFolder, 'src', 'perception', 'ground_truth', 'mat');
    else
        error('Unknown project: %s', proj.Name);
    end
end
