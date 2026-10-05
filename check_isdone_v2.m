function isdone = check_isdone_v2(x, y, pathXY)
% CHECK_ISDONE_V2  Episode selesai jika robot sudah dekat titik AKHIR path.
    end_pt = pathXY(end, :);
    dist = sqrt((end_pt(1)-x)^2 + (end_pt(2)-y)^2);
    if dist < 0.5
        isdone = 1;
    else
        isdone = 0;
    end
end