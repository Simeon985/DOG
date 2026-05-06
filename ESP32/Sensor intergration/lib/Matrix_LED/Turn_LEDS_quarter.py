def rotate(matrix: list[int], n: int) -> list[int]:
    n = n % 4
    for _ in range(n):
        bits = [[( row >> (7 - col)) & 1 for col in range(8)] for row in matrix]
        matrix = [
            sum(bits[7 - row][col] << (7 - row) for row in range(8))
            for col in range(8)
        ]
    return matrix
matrix = [  0b00000000,
  0b00011100,
  0b00111110,
  0b01101011,
  0b01000001,
  0b01100011,
  0b00110110,
  0b00011100]
print(rotate(matrix, 1))