%% Visualize pose data in WebMap

data = importdata("Mission 1.txt",' ',0);

webmap('World Imagery')
wmmarker(data(1, 6), data(1, 7))

wmmarker()
wmline(data(:, 6), data(:, 7), 'Color', 'r')

% show the track in a TM coordinate system
e = referenceEllipsoid('wgs84');
ax = axesm('MapProjection', 'tranmerc', 'Geoid', e, 'MapLatLimit', [37.5 37.6], 'MapLonLimit', [127.0 127.1], ...
    'Origin', [38 127], 'FalseEasting', 200000, 'FalseNorthing', 600000, ...
    'MeridianLabel', 'on', 'ParallelLabel', 'on', ...
    'PLineLocation', [37.5:0.02:37.6], 'MLineLocation', [127.0:0.02:127.1], ...
    'PLabelLocation', [37.5:0.02:37.6], 'MLabelLocation', [127.0:0.02:127.1], ...
    'PLabelRound', -2, 'MLabelRound', -2, ...
    'Frame', 'on', 'Grid', 'on');
geoshow(trk.Latitude, trk.Longitude);

% convert the lat. and long into x, y in TM
mstruct = getm(ax);
[x,y] = mfwdtran(mstruct, trk.Latitude, trk.Longitude);
pos_tm = [x', y']

pos_tm(1:10, :)

% Compute the mean and standard deviation
mean(pos_tm)
std(pos_tm)