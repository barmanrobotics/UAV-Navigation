import matplotlib.pyplot as plt
import pandas as pd

def plot_csv_columns(filename):
    # Read the CSV file
    data = pd.read_csv(filename)

    # Filter only numeric columns
    numeric_data = data.select_dtypes(include='number')
    column_names = numeric_data.columns

    if numeric_data.empty:
        print("CSV file does not contain any numeric columns to plot.")
        return

    num_cols = len(column_names)
    
    # Create subplots
    fig, axes = plt.subplots(num_cols, 1, figsize=(10, 3 * num_cols), squeeze=False)
    axes = axes.flatten()
    scaled_x = numeric_data.index * 0.1

    # Define basic color/style cycles
    colors = plt.cm.get_cmap('tab10', num_cols)
    line_styles = ['-', '--', '-.', ':']
    markers = ['o', 's', '^', 'd', 'x', '*', '', '+']

    for i, col in enumerate(column_names):
        color = colors(i % 10)
        linestyle = line_styles[i % len(line_styles)]
        marker = markers[i % len(markers)]

        axes[i].plot(scaled_x, numeric_data[col], label=col, color=color, linestyle=linestyle)
        axes[i].set_title(f'{col} over Time')
        axes[i].set_ylabel(col)
        axes[i].legend()
        axes[i].grid(True)

    axes[-1].set_xlabel('Time (scaled index)')
    plt.tight_layout()
    plt.show()

# Ask the user for the filename
filename = input("Please enter the filename of the CSV file: ")
plot_csv_columns(filename)
