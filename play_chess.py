
class Chess:

    def __init__(self):
        self.human = "X"    #两个不同的棋子
        self.computer="Y"
        self.board=[" "]*9
    def legal_moves(self):
    #返回空白位置
        moves=[]
        for i in range(9):
            if self.board[i]==" ":
                moves.append(i)
        return moves
    def display_board(self):
    #显示整个棋盘
        board2=self.board[:]
        for i in range(len(self.board)):
            if self.board[i]==" ":
                board2[i]=i
        print("\t",board2[0],"|",board2[1],"|",board2[2])
        print("\t","----------")
        print("\t",board2[3],"|",board2[4],"|",board2[5])
        print("\t","----------")
        print("\t",board2[6],"|",board2[7],"|",board2[8])

    def winner(self,boards):
    #返回赢家
        waystowin=((0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(2,5,8),(0,4,8),(2,4,6))#几种赢的方式
        for row in waystowin:
            if  boards[row[0]]==boards[row[1]]==boards[row[2]]!=" ":
                winner=boards[row[0]]
                return winner
        if " " not in boards:
            return "平局"
        return False
    #计算下一步怎么走
    def ask_number(self,question, low, high):
        response = None
        while response not in range(low, high):
            response = int(input(question))
        return response
    def human_move(self):
        legal=self.legal_moves()
        move=None
        while move not in legal:
            move=self.ask_number("where to go？（0-8）：",0,9)
            if move not in legal:
                print("\nThere is already a chess")
        return move


    def computer_move(self):

        board2=self.board[:]
        bestmove=(4,0,2,6,8,1,3,5,7)#最佳下棋位置顺序表
        legalmove=self.legal_moves()#获取空白位置
        for move in legalmove:
        #如果计算机能赢就走这里
            board2[move]=self.computer
            if self.winner(board2)==self.computer:
                print("Go here:", move)
                return move
            board2[move]=" "
        for move in legalmove:
            #如果玩家赢就走这里
            board2[move]=self.human
            if self.winner(board2)==self.human:
                print("Go here:", move)
                return move
            board2[move]=" "
        for move in bestmove:
            #一步不能赢就选最好的方式走
            if move in legalmove:
                print("Go here:", move)
                return move
    def next_turn(self,turn):
        if turn=="X":
            return "Y"
        else:
            return "X"


if __name__ == '__main__':
    chess=Chess()
    chess.display_board()
    turn=chess.human
    while not chess.winner(chess.board):
        if turn==chess.human:
            move=chess.human_move()
            chess.board[move]=chess.human
        else:
            move=chess.computer_move()
            chess.board[move]=chess.computer
        chess.display_board()
        turn=chess.next_turn(turn)
        the_winner=chess.winner(chess.board)
        if the_winner == chess.computer:
            print("计算机赢！\n")
        elif the_winner == chess.human:
            print("玩家赢！\n")
        elif the_winner == "TIE":
            print("平局，游戏结束\n")


