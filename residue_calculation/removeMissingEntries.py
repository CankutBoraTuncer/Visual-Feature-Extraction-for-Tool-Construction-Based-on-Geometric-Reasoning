import os
import pandas as pd

def process_folders(folder1, folder2, output_folder):
    # Ensure the output folder exists
    os.makedirs(output_folder, exist_ok=True)

    # List all files in both folders
    folder1_files = set(os.listdir(folder1))
    folder2_files = set(os.listdir(folder2))

    # Find matching files by name
    common_files = folder1_files.intersection(folder2_files)

    for file_name in common_files:
        file1_path = os.path.join(folder1, file_name)
        file2_path = os.path.join(folder2, file_name)
        output_path = os.path.join(output_folder, file_name)

        try:
            # Read the files
            df1 = pd.read_csv(file1_path, names=["Point Cloud Name", "Base Residue Scores"], skiprows=1)
            df2 = pd.read_csv(file2_path, names=["Point Cloud Name", "Our Residue Scores"], skiprows=1)


            # Merge the DataFrames with suffixes to handle overlapping columns
            merged_df = pd.merge(df1, df2, on="Point Cloud Name", suffixes=("_file1", "_file2"))

            # Remove rows with missing values
            cleaned_df = merged_df.dropna()

            # Save the cleaned DataFrame to the output folder
            cleaned_df.to_csv(output_path, index=False)
            print(f"Processed and saved: {output_path}")
        except Exception as e:
            print(f"Error processing files {file1_path} and {file2_path}: {e}")

# Example usage
folder1 = "/Users/hsimsir/Documents/CS554_Computer_Vision_Project/base_residue_scores"
folder2 = "/Users/hsimsir/Documents/CS554_Computer_Vision_Project/our_residue_scores"
output_folder = "combined_residue_scores"

process_folders(folder1, folder2, output_folder)
