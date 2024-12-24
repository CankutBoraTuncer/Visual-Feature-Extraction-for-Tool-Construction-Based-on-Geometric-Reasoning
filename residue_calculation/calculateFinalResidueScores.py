import os
import pandas as pd

def process_files_by_extension(directory):
    # Specify file extensions to process
    file_extensions = ['0.txt', '1.txt', '2.txt', '3.txt']
    results = {ext: {"base_residue_total": 0, "our_residue_total": 0, "total_rows": 0} for ext in file_extensions}

    # Iterate through all files in the directory
    for file_name in os.listdir(directory):
        for ext in file_extensions:
            if file_name.endswith(ext):
                file_path = os.path.join(directory, file_name)
                try:
                    # Read the file into a DataFrame
                    df = pd.read_csv(file_path)

                    # Update totals for the current extension
                    results[ext]["base_residue_total"] += df["Base Residue Scores"].sum()
                    results[ext]["our_residue_total"] += df["Our Residue Scores"].sum()
                    results[ext]["total_rows"] += len(df)
                except Exception as e:
                    print(f"Error processing file {file_path}: {e}")

    # Print the average scores for each file extension
    for ext, data in results.items():
        if data["total_rows"] > 0:
            avg_base = data["base_residue_total"] / data["total_rows"]
            avg_our = data["our_residue_total"] / data["total_rows"]
            print(f"SQ type  {ext[0]}:")
            print(f"  Average Base Residue Scores: {avg_base:.4f}")
            print(f"  Average Our Residue Scores: {avg_our:.4f}")
        else:
            print(f"No valid files processed for extension {ext}.")

# Example usage
directory = "./combined_residue_scores"
process_files_by_extension(directory)
