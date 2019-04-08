b = Bluetooth('hank',1);
fopen(b);

data_length = 500;
data = zeros(data_length,2);
for i = 1:data_length
   data(i,1) = str2double(fscanf(b));
   data(i,2) = str2double(fscanf(b));
end

plot(data)

fclose(b);
