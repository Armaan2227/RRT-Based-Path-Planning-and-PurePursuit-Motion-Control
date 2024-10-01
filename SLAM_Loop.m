% dist_new = (DISTANCE)';
% head_new = (HEADING)';


% for i=1:240
%     dist1 = dist_new(i,:);
%     head1 = head_new(i,:);
%     scan = lidarScan(dist1,head1);
%     scansCopy{i} = scan;
% end

for i=1:length(dist_lida.time)
    dist2 = dist_lida.signals.values(i,:);
    head2 = head_lida.signals.values(i,:);
    scan1 = lidarScan(dist2,head2);
    scansCopy{i} = scan1;
end
