void initMaze(){
    for(int row=0; row<mazeSize; row++)
        for(int column=0; column<mazeSize; column++)
            maze[row][column] = "000000";
}

int countAvailableMovements(){
    int moves=0;
    if(maze[actualRow][actualCol][0]=='0' && comeFrom!='W'){   //Will move West
        if(maze[actualRow][actualCol-1][4]=='0')
            moves++;
    }
    if(maze[actualRow][actualCol][1]=='0' && comeFrom!='N'){   //Will move North
        if(maze[actualRow-1][actualCol][4]=='0')
            moves++;
    }
    if(maze[actualRow][actualCol][2]=='0' && comeFrom!='E'){   //Will move East
        if(maze[actualRow][actualCol+1][4]=='0')
            moves++;
    }
    if(maze[actualRow][actualCol][3]=='0' && comeFrom!='S'){   //Will move South
        if(maze[actualRow+1][actualCol][4]=='0')
            moves++;
    }
    return moves;
}

void addFatherStack(StackArray<char> &generalStack, StackArray<char> &fatherStack){
    generalStack.push('#');
    fatherStack.push('#');
    if(maze[actualRow][actualCol][3]=='0' && comeFrom!='S'){   //Will move South
        if(maze[actualRow+1][actualCol][4]=='0')
            fatherStack.push('S');
    }
    if(maze[actualRow][actualCol][2]=='0' && comeFrom!='E'){   //Will move East
        if(maze[actualRow][actualCol+1][4]=='0')
            fatherStack.push('E');
    }
    if(maze[actualRow][actualCol][1]=='0' && comeFrom!='N'){   //Will move North
        if(maze[actualRow-1][actualCol][4]=='0')
            fatherStack.push('N');
    }
    if(maze[actualRow][actualCol][0]=='0' && comeFrom!='W'){   //Will move West
        if(maze[actualRow][actualCol-1][4]=='0')
            fatherStack.push('W');
    }
}

void addGeneralStack(StackArray<char> &generalStack){
    if(maze[actualRow][actualCol][0]=='0' && comeFrom!='W'){   //Will move West
        if(maze[actualRow][actualCol-1][4]=='0')
            generalStack.push('W');
    }
    else if(maze[actualRow][actualCol][1]=='0' && comeFrom!='N'){   //Will move North
        if(maze[actualRow-1][actualCol][4]=='0')
            generalStack.push('N');
    }
    else if(maze[actualRow][actualCol][2]=='0' && comeFrom!='E'){   //Will move East
        if(maze[actualRow][actualCol+1][4]=='0')
            generalStack.push('E');
    }
    else if(maze[actualRow][actualCol][3]=='0' && comeFrom!='S'){   //Will move South
        if(maze[actualRow+1][actualCol][4]=='0')
           generalStack.push('S');
   }
}

char inverseDirection(char direction){
    if(direction == 'W')   //Will move West
        return 'E';
    else if(direction == 'N')   //Will move North
        return 'S';
    else if(direction == 'E')   //Will move East
        return 'W';
    else if(direction == 'S')   //Will move South
        return 'N';
}
void addInverseStack(StackArray<char> generalStack, StackArray<char> &inverseStack){
    StackArray <char>auxStack;
    while(generalStack.peek()!='#'){
        auxStack.push(generalStack.peek());
        generalStack.pop();
    }
    while(!auxStack.isEmpty()){
        inverseStack.push(inverseDirection(auxStack.peek()));
        auxStack.pop();
    }
}

void move(char direction){
    maze[actualRow][actualCol][4] = '1';
    if(direction=='W'){   //Will move West
        actualCol--;
        comeFrom = 'E';
    }
    else if(direction=='N'){   //Will move North
        actualRow--;
        comeFrom = 'S';
    }
    else if(direction=='E'){   //Will move East
        actualCol++;
        comeFrom = 'W';
    }
    else if(direction=='S'){   //Will move South
        actualRow++;
        comeFrom = 'N';
    }
}

bool robotMovement(StackArray<char> &generalStack, StackArray<char> &fatherStack, StackArray<char> &inverseStack, String bits){
    maze[actualRow][actualCol]=bits;
    int nMovements = countAvailableMovements();
    if(nMovements == 0){
        if(generalStack.peek()=='#' && fatherStack.peek()=='#'){
            generalStack.pop();
            fatherStack.pop();
        }
        if(!fatherStack.isEmpty()){
            addInverseStack(generalStack, inverseStack);
            if(inverseStack.isEmpty()){
                fatherStack.pop();
            }
            while(!inverseStack.isEmpty()){//Returns to last father
                move(inverseStack.peek());
                inverseStack.pop();
                generalStack.pop();
            }
        }
        else{
            return true;//Algorith DONE
        }
    }
    else{
        if(nMovements > 1)
            addFatherStack(generalStack, fatherStack);
        if(generalStack.peek()=='#' && fatherStack.peek()!='#'){
            generalStack.push(fatherStack.peek());
            fatherStack.pop();
        }
        else{
            addGeneralStack(generalStack);
        }
        move(generalStack.peek());
    }
    return false;
}

void mazeAlgorithm(){
    char casilla;
    bool done=false;
    String bits;
    StackArray <char> fatherStack;
    StackArray <char> generalStack;
    StackArray <char> inverseStack;
    initMaze();
    do{
        //Read Sensors
        done = robotMovement(generalStack, fatherStack, inverseStack, bits);
    }while(!done);
}

