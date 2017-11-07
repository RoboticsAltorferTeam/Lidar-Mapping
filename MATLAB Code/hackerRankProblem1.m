% Inputs
num_days = 5;
demands = [1 2 3 2 1];
max_capacity = 5;
overnight_cost = 1;
shipping_cost = 20;
price = 1;

% Solution
current_stock = 0;
free_storage = max_capacity;
total_demands = sum(demands);
total_cost = 0;

for i = 1:num_days
    buy_today = 0;
    for j = current_stock:free_storage
        if(j > demands(i))
end