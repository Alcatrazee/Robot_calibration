function result = vee(input)

if(length(input)==3)
   result = zeros(3,1);
   result(1) = input(3,2);
   result(2) = input(1,3);
   result(3) = input(2,1);
elseif(length(input)==4)
   result = zeros(6,1);
   result(1:3) = vee(input(1:3,1:3));
   result(4) = input(1,4);
   result(5) = input(2,4);
   result(6) = input(3,4);
end