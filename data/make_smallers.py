import sys

def generate_test(num_lines, F_lines, fname):
    with open(f"./testset/{fname}", "w") as f:
        header_count = F_lines[2].split()
        header_count[2] = str(num_lines) + '\n'
        F_lines[2] = ' '.join(header_count)
        f.writelines(F_lines[:num_lines + 7])


def main(to_gen):
    file_A = open("./livingroom_dense_A.ply", "r")
    file_B = open("./livingroom_dense_B.ply", "r")
    A_lines = file_A.readlines()
    B_lines = file_B.readlines()
    file_A.close()
    file_B.close()
    for num_lines in to_gen:
        num_lines = int(num_lines)
        generate_test(num_lines*1000, A_lines, f"{num_lines}A")
        generate_test(num_lines*1000, B_lines, f"{num_lines}B")

if __name__ == '__main__':
    assert len(sys.argv) >= 2
    main(sys.argv[1:])