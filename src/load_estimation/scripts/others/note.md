- data source: /home/iwa/wheel_loader_ws/results/csv/all.csv
- modifiy file: /home/iwa/wheel_loader_ws/src/load_estimation/scripts/others/regression.py

## intruction
1. extract value of the collumn "/debug_geometry/median_pb", "/debug_geometry/median_pr" and "/debug_geometry/theta_g" when collumn "omega_t" and "omega_g" have magnitude less than 0.01
2. find trend line (regression) between extracted median_pb and omega_g from step 1 (hint: use polinomial degree 2)
3. find trend line (regression) between extracted median_pr and omega_g from step 1
4. write python code in modifiy file

## goal 
- function pb of theta_g and show value of R^2
- function pr of theta_g and show value of R^2 
- graph plot pb(vertical) and theta_g(horizontal)
- graph plot pr(vertical) and theta_g(horizontal)