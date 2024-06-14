rm *.ply

python3 ../csv_to_ply.py all_transformed_vertices.csv all_transformed_vertices.csv.ply
python3 ../csv_to_ply.py ./scan.csv ./scan.csv.ply 
python3 ../csv_to_ply.py ./cad_nypos.csv ./cad_nypos.csv.ply 
python3 ../csv_to_ply.py ./cad.csv ./cad.csv.ply 
