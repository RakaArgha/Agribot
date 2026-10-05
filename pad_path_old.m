function pathXY_padded = pad_path(pathXY, max_size)
% PAD_PATH  Mengubah path jadi ukuran TETAP (wajib untuk MATLAB Function block).
% Titik terakhir diulang untuk mengisi sampai max_size baris.
    M = size(pathXY, 1);
    if M >= max_size
        pathXY_padded = pathXY(1:max_size, :);
    else
        last_pt = pathXY(end, :);
        pad = repmat(last_pt, max_size - M, 1);
        pathXY_padded = [pathXY; pad];
    end
end