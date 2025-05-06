import tkinter as tk
import numpy as np

def manually_select_chess_pieces():
    board_size = 8
    cell_size = 60
    piece_states = [0, 1, -1]  # empty, white, black
    state_colors = {0: 'green', 1: 'white', -1: 'black'}

    root = tk.Tk()
    root.title("Select Chess Pieces")

    board_state = np.zeros((board_size, board_size), dtype=int)
    buttons = [[None for _ in range(board_size)] for _ in range(board_size)]

    def cycle_state(i, j):
        current_state = board_state[i, j]
        next_state_index = (piece_states.index(current_state) + 1) % len(piece_states)
        board_state[i, j] = piece_states[next_state_index]
        buttons[i][j].config(bg=state_colors[board_state[i, j]])

    for i in range(board_size):
        for j in range(board_size):
            btn = tk.Button(root, bg=state_colors[0], width=4, height=2,
                            command=lambda i=i, j=j: cycle_state(i, j))
            btn.grid(row=i, column=j)
            buttons[i][j] = btn

    def finish():
        root.quit()
        root.destroy()

    finish_button = tk.Button(root, text="Finish", command=finish)
    finish_button.grid(row=board_size, column=0, columnspan=board_size, sticky="ew")

    root.mainloop()
    return board_state

# Example usage
board = manually_select_chess_pieces()
print("Final board:")
print(board)
