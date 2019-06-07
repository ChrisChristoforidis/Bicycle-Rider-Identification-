function en = statefbError2(X,np,bike_m, dat)

K=X;
out = modelSim(K,bike_m,dat);
e = (out.steer_angle - np.y(:,2));
en = ((sum(e.^2)) * 1 / np.N);
en=mean(en);

end