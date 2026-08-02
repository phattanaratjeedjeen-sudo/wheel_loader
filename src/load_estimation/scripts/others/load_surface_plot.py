import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score

'''
find: est_load(w, theta_g)
where
- est_load(ton)
- w(kg): load after enable empty bucket offset 
- theta_g(rad): raw value from encoder
'''

def find_surface_equation(df_long):
    print("\n--- Finding Best Surface Equation ---")
    
    # We want load as a function of (w, theta_g)
    X = df_long[['w', 'theta_g']].values
    y = df_long['load_numeric'].values
    
    best_degree = 1
    best_r2 = -float('inf')
    best_rmse = float('inf')
    best_model = None
    best_poly = None
    
    # Test polynomial degrees 1 through 5 to find the sweet spot
    for degree in range(1, 6):
        poly = PolynomialFeatures(degree=degree, include_bias=False)
        X_poly = poly.fit_transform(X)
        
        model = LinearRegression()
        model.fit(X_poly, y)
        
        y_pred = model.predict(X_poly)
        r2 = r2_score(y, y_pred)
        rmse = np.sqrt(mean_squared_error(y, y_pred))
        
        print(f"Testing Degree {degree}: R^2 = {r2:.5f}, RMSE = {rmse:.5f}")
        
        if r2 > best_r2:
            best_r2 = r2
            best_rmse = rmse
            best_model = model
            best_poly = poly
            best_degree = degree
            
        # Stop early if we hit an excellent fit (prevents overfitting to noise)
        if r2 > 0.999:
            break
            
    print(f"\nBest Fit achieved with Polynomial Degree {best_degree}")
    print(f"R^2:   {best_r2:.5f}")
    print(f"RMSE:  {best_rmse:.5f}")
    
    # Build a readable equation string
    feature_names = best_poly.get_feature_names_out(['w', 'theta_g'])
    coefs = best_model.coef_
    intercept = best_model.intercept_
    
    equation = f"load(w, theta_g) = {intercept:.4f}"
    
    for name, coef in zip(feature_names, coefs):
        # Format the terms to look like standard math (e.g., w * theta_g)
        name = name.replace(' ', ' * ').replace('^2', '**2').replace('^3', '**3').replace('^4', '**4').replace('^5', '**5')
        
        if coef >= 0:
            equation += f" + {coef:.4e} * {name}"
        else:
            equation += f" - {abs(coef):.4e} * {name}"
            
    print("\n--- Final Surface Equation ---")
    print(equation)
    print("------------------------------\n")
    
    return best_model, best_poly

def main():
    # 1. Define the filename exactly as requested
    path = os.path.expanduser('~/wheel_loader_ws/results/csv/settle/')
    filename = "estimate load - geometry.csv"
    filename = os.path.join(path, filename)
    
    # Check if file exists before trying to read it
    if not os.path.exists(filename):
        print(f"Error: The file '{filename}' was not found in the current directory.")
        print("Please ensure the script and the CSV file are in the same folder.")
        return

    # 2. Read the CSV data
    print(f"Reading data from {filename}...")
    df = pd.read_csv(filename)
    
    # Print the columns to help with debugging if names don't match exactly
    print("Columns found in CSV:", df.columns.tolist())
    
    # 3. Reshape the data from wide to long format
    # The CSV has 'theta_g' as a column, and loads as separate columns (e.g., '0.0t')
    if 'theta_g' not in df.columns:
        print("Error: 'theta_g' column not found.")
        return

    # Find all columns that look like load columns (e.g., end with 't' and start with a digit)
    load_cols = [col for col in df.columns if col.endswith('t') and col[0].isdigit()]
    
    if not load_cols:
        print("Error: No load columns (e.g., '0.1t') found in the CSV.")
        return

    print(f"Found {len(load_cols)} load columns. Reshaping data...")

    # Melt the dataframe so that load columns become rows
    # This turns wide grid format into ['theta_g', 'load_str', 'w'] coordinate pairs
    df_long = df.melt(id_vars=['theta_g'], value_vars=load_cols, 
                      var_name='load_str', value_name='w')

    # Drop any rows where 'w' or 'theta_g' is missing (empty cells in the CSV)
    df_long = df_long.dropna(subset=['theta_g', 'w'])

    # Clean the 'load_str' column (e.g. "1.66t") by stripping the 't' and casting to float
    df_long['load_numeric'] = df_long['load_str'].astype(str).str.replace(r'[^\d.]', '', regex=True).astype(float)
    
    # Run our surface fitting function to find the mathematical equation
    find_surface_equation(df_long)
    
    # Extract the variables for plotting
    x = df_long['theta_g']
    y = df_long['w']
    z = df_long['load_numeric']

    # 4. Create the 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot a triangulated surface (best for scattered x, y, z data)
    surf = ax.plot_trisurf(x, y, z, cmap='viridis', edgecolor='none', alpha=0.85)
    
    # Overlay the actual data points as black dots for reference
    ax.scatter(x, y, z, color='black', s=20, label='Actual Data Points')

    # 5. Format the plot
    ax.set_title('Load Surface Model: load(theta_g, w)', fontsize=14, pad=20)
    ax.set_xlabel('theta_g (rad)', fontsize=12)
    ax.set_ylabel('w', fontsize=12)
    ax.set_zlabel('Load (tons)', fontsize=12)
    
    # Add a color bar mapping to the load values
    cbar = fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, pad=0.1)
    cbar.set_label('Load (t)')
    
    plt.legend()
    plt.tight_layout()
    
    # Show the interactive plot window
    print("Generating plot...")
    plt.show()

if __name__ == "__main__":
    main()