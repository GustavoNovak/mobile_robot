cameras = classes.Cameras();

tic
for i = 1:200
    snapshot = getsnapshot(cameras.cameras(2));
    imshow(snapshot);
end

elapsedTime = toc

% Compute the time per frame and effective frame rate.
timePerFrame = elapsedTime/200
effectiveFrameRate = 1/timePerFrame

cameras.delete();