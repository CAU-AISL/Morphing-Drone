class ModeController:
    def __init__(self, state):
        self.state = state  

    def update_abw(self):
    # self.state.x_hat 이런식으로 값 가져와서 계산한 다음에 아래처럼 값 바꾸면 됨!
    # self.state 쓰는거 귀찮으면 init 안에 x_hat = self.state.x_hat 이런식으로 추가하면 됨 근데 이렇게 하면 main에 반영은 안됨
    
        if self.state.mode == 'X':
            self.state.alpha = 0.0
            self.state.beta = 0.0
            self.state.w_d = 0.0
        
        elif self.state.mode == 'Y':
            self.state.alpha = 0.0
            self.state.beta = 0.0
            self.state.w_d = 0.0
            
        else:  # 'H'
            self.state.alpha = 0.0
            self.state.beta = 0.0
            self.state.w_d = 0.0
      
