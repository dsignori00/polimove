function clouds = pointCloud2TableToCellArray(log, topic, fields_to_keep)

    S = log.(topic);

    fields_to_keep = string(fields_to_keep);

    n_msg = size(S.data, 1);
    clouds = cell(n_msg, 1);

    % PointCloud2 field metadata.
    % We assume the schema is identical for every message.
    field_offsets   = double(S.fields__offset(1, :));
    field_datatypes = double(S.fields__datatype(1, :));
    field_counts    = double(S.fields__count(1, :));

    n_fields = numel(field_offsets);

    % fields__name is flattened into one char row.
    % In this dataset there are 7 fields and 63 chars -> 9 chars per name.
    names_raw = S.fields__name(1, :);
    chars_per_field = size(S.fields__name, 2) / n_fields;

    field_names = strings(1, n_fields);

    for k = 1:n_fields
        idx = (k-1)*chars_per_field + (1:chars_per_field);

        name = names_raw(idx);

        % Remove null padding and whitespace.
        name(name == char(0)) = [];
        field_names(k) = strtrim(string(name));
    end

    for i = 1:n_msg
        
        header_stamp_s = double(S.header__stamp__tot(i));

        n_points = double(S.width(i)) * double(S.height(i));
        point_step = double(S.point_step(i));

        data = uint8(S.data(i, 1:n_points * point_step));
        data = reshape(data, point_step, n_points);

        cloud = nan(n_points, numel(fields_to_keep));

        for j = 1:numel(fields_to_keep)
            
            is_timestamp = fields_to_keep(j) == "timestamp";
            field_idx = find(field_names == fields_to_keep(j), 1);

            if isempty(field_idx)
                error( ...
                    'pointCloud2TableToCellArray:MissingField', ...
                    'Field "%s" not found. Available fields: %s', ...
                    fields_to_keep(j), ...
                    strjoin(field_names, ', '));
            end

            if field_counts(field_idx) ~= 1
                error( ...
                    'pointCloud2TableToCellArray:UnsupportedCount', ...
                    'Field "%s" has count = %d.', ...
                    fields_to_keep(j), ...
                    field_counts(field_idx));
            end

            offset = field_offsets(field_idx);
            datatype = field_datatypes(field_idx);

            [matlab_type, n_bytes] = pointFieldType(datatype);

            % ROS PointField.offset is zero-based.
            first_byte = offset + 1;
            last_byte  = first_byte + n_bytes - 1;

            raw = data(first_byte:last_byte, :);

            values = typecast(reshape(raw, 1, []), matlab_type);

            cloud(:, j) = double(values(:));
            if(is_timestamp)
                cloud(:, j) = double(cloud(:, j)*1e-6) + header_stamp_s;
            end
        end

        % Preserve your previous behaviour:
        % reject a point if any requested field is non-finite.
        valid = all(isfinite(cloud), 2);

        clouds{i} = cloud(valid, :);
    end
end


function [matlab_type, n_bytes] = pointFieldType(datatype)

    % sensor_msgs/msg/PointField constants:
    %
    % INT8    = 1
    % UINT8   = 2
    % INT16   = 3
    % UINT16  = 4
    % INT32   = 5
    % UINT32  = 6
    % FLOAT32 = 7
    % FLOAT64 = 8

    switch datatype
        case 1
            matlab_type = 'int8';
            n_bytes = 1;

        case 2
            matlab_type = 'uint8';
            n_bytes = 1;

        case 3
            matlab_type = 'int16';
            n_bytes = 2;

        case 4
            matlab_type = 'uint16';
            n_bytes = 2;

        case 5
            matlab_type = 'int32';
            n_bytes = 4;

        case 6
            matlab_type = 'uint32';
            n_bytes = 4;

        case 7
            matlab_type = 'single';
            n_bytes = 4;

        case 8
            matlab_type = 'double';
            n_bytes = 8;

        otherwise
            error('Unsupported PointField datatype: %d', datatype);
    end
end