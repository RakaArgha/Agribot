function isdone = check_isdone_v3(x, y, pathXY)
% CHECK_ISDONE_V3  Radius selesai diperketat dari 0.5 -> 0.2 meter.
    end_pt = pathXY(end, :);
    dist = sqrt((end_pt(1)-x)^2 + (end_pt(2)-y)^2);
    if dist < 0.2
        isdone = 1;
    else
        isdone = 0;
    end
end