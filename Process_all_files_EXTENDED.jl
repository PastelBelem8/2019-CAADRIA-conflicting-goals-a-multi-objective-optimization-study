using DelimitedFiles

base_dir = "final/Analises"
algorithms = ["OMOPSO", "SMPSO", "MOEAD", "SPEA2", "EpsMOEA", "PESA2", "PAES", "NSGAII", "CMAES", "GDE3"]
runs = [1, 2, 3]

get_filename(a, r) = "$(base_dir)/$(a)_results_0$(r).csv"

filenames = map(c -> get_filename(c[1], c[2]), Iterators.product(algorithms, runs))
filenames = filter(isfile, filenames)
println("About to join the following files:\n", join(filenames, '\n'))

read_files(;filename, header=true) = open(filename, "r") do f
                                   header ? readline(f) : nothing
                                   readdlm(f, ',')
                               end

read_cols(; filename, cols, header=true) = read_files(filelanme=filename, header=header)[:, cols]

# Sanity check for the operations above
println("Reading file: $(filenames[1])")
f1_data = read_files(;filename=filenames[1])
println("Size of collected data: $(size(f1_data))")
println("Type of collected data: $(typeof(f1_data))")

# Add more files to parse
filenames = [filenames
             "final/SPEA2_2000.csv"
             "final/SPEA2_4000.csv"
             "SPEA2_results_01.csv"
             "SMPSO_results_01.csv"
             "SMPSO_results_02.csv"
             "SMPSO_results_03.csv"
             "SMPSO_results_04.csv"
             "NSGAII_results_40ps_20iter.csv"
             "NSGAII_results_100ps_90iter.csv"
             ]


smpso_extra = map(c -> "$(c[1])/SMPSO_results_run$(c[2]).csv",
                    Iterators.product([ "SMPSO_Swarm15_leader5_maxeval1",
                                        "SMPSO_Swarm15_leader5_maxeval5",
                                        "SMPSO_Swarm15_leader5_maxeval10",
                                        "SMPSO_Swarm15_leader5_maxeval15",
                                        "SMPSO_Swarm15_leader5_maxeval20",
                                        "SMPSO_Swarm15_leader5_maxeval50"], runs))

filenames = vcat(filenames, smpso_extra[:])
filenames = filter(isfile, filenames)

output_folder = "final/outputs"
output_file = "$(output_folder)/all_solutions_20190310.csv"
println("Concatenating all files in a single one called `$(output_file)`...")
all_data = vcat(map(f -> read_files(filename=f, header=true), filenames)...)


writedlm(output_file, all_data,',')
