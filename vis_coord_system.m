function vis_coord_system (pos, rot, len, coord_name, cstr)

% rot : 각 축에 대한 단위벡터 3x3
% 1행 : 지상좌표계 기준 카메라 좌표계의 x축 단위벡터

% pos : 지상좌표계 기준 카메라 원점의 (x, y, z) 좌표
% len : 축의 길이


cs = {'r', 'g', 'b'};

for n = 1:3
    cstr = cs{n};
    pt = pos + len * rot(:,n);
    hold on
    h = plot3([pos(1), pt(1)], [pos(2), pt(2)], [pos(3), pt(3)], 'r-');
    set(h, 'LineWidth', 2, 'Color', cstr);
    h = text( pt(1),  pt(2),  pt(3), sprintf('%c', n+'x'-1) );
    set(h, 'Color', cstr, 'FontWeight', 'bold', 'FontSize', 9, ...
         'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');
end

cstr = 'k';
h = text( pos(1),  pos(2),  pos(3), coord_name );
set(h, 'Color', cstr, 'FontWeight', 'bold', 'FontSize', 9, ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');

