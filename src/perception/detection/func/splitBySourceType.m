function [split_structs, source_types] = splitBySourceType(data)

    % Different source_type values
    source_types = unique(data.source_type);
    n_sources = numel(source_types);

    % One struct per source_type
    split_structs = cell(n_sources, 1);

    n_entries = numel(data.source_type);
    fields = fieldnames(data);

    for i = 1:n_sources

        source = source_types(i);
        idx = data.source_type == source;

        out = struct();

        for j = 1:numel(fields)

            field_name = fields{j};
            value = data.(field_name);

            % Fields containing one entry per detection/frame
            if ~isempty(value) && size(value, 1) == n_entries
                out.(field_name) = value(idx, :);
            else
                % Scalar/global fields, e.g. max_det
                out.(field_name) = value;
            end
        end

        split_structs{i} = out;
    end
end