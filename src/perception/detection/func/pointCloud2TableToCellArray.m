function clouds = pointCloud2TableToCellArray(log, topic)

    S = log.(topic);

    n_msg = size(S.data, 1);
    clouds = cell(n_msg, 1);

    for i = 1:n_msg

        n_points = double(S.width(i)) * double(S.height(i));
        point_step = double(S.point_step(i));

        data = uint8(S.data(i, 1:n_points * point_step));
        data = reshape(data, point_step, n_points);

        x = typecast(reshape(data(1:4, :), 1, []), 'single');
        y = typecast(reshape(data(5:8, :), 1, []), 'single');

        valid = isfinite(x) & isfinite(y);

        clouds{i} = double([x(valid)', y(valid)']);
    end
end