#include <iostream>
#include <fstream>
#include <stack>
#include <string>
using namespace std;

const int mazeSize=9;
int actualRow=8, actualCol=6;
char comeFrom='S';
string maze[mazeSize][mazeSize];

void initMaze(){
    for(int row=0; row<mazeSize; row++)
        for(int column=0; column<mazeSize; column++)
            maze[row][column] = "000000";
}

void printMaze(){
    for(int row=0; row<mazeSize; row++){
        for(int column=0; column<mazeSize; column++){
            if(row==actualRow && column==actualCol)
                cout << "###### ";
            else
                cout << maze[row][column] << " ";
        }
        cout << endl;
    }
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

void addFatherStack(stack<char> &generalStack, stack<char> &fatherStack){
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

void addGeneralStack(stack<char> &generalStack){
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
void addInverseStack(stack<char> generalStack, stack<char> &inverseStack){
    stack <char>auxStack;
    while(generalStack.top()!='#'){
        auxStack.push(generalStack.top());
        generalStack.pop();
    }
    while(!auxStack.empty()){
        inverseStack.push(inverseDirection(auxStack.top()));
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

void showCoordinates(){
    cout << "Row = " << actualRow << "\t\tColumn = " << actualCol << endl;
}

void showStack(stack<char> printStack){
    stack<char> auxStack;
    while(!printStack.empty()){
        auxStack.push(printStack.top());
        printStack.pop();
    }
    while(!auxStack.empty()){
        cout << auxStack.top() << " ";
        auxStack.pop();
    }
    cout << endl;
}
void showAllStacks(stack<char> generalStack, stack<char> fatherStack, stack<char> inverseStack){
    cout << "GeneralStack:" << endl;
    showStack(generalStack);
    cout << "FatherStack:" << endl;
    showStack(fatherStack);
    cout << "InverseStack:" << endl;
    showStack(inverseStack);
}

bool robotMovement(stack<char> &generalStack, stack<char> &fatherStack, stack<char> &inverseStack, string bits){
    maze[actualRow][actualCol]=bits;
    int nMovements = countAvailableMovements();
    if(nMovements == 0){
        if(generalStack.top()=='#' && fatherStack.top()=='#'){
            generalStack.pop();
            fatherStack.pop();
        }
        if(!fatherStack.empty()){
            addInverseStack(generalStack, inverseStack);
            if(inverseStack.empty()){
                fatherStack.pop();
            }
            while(!inverseStack.empty()){//Returns to last father
                move(inverseStack.top());
                showCoordinates();
                showStack(inverseStack);
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
        if(generalStack.top()=='#' && fatherStack.top()!='#'){
            generalStack.push(fatherStack.top());
            fatherStack.pop();
        }
        else{
            addGeneralStack(generalStack);
        }
        move(generalStack.top());
    }
    return false;
}

int main(){
    char casilla;
    bool done=false;
    string bits;
    stack<char> fatherStack;
    stack<char> generalStack;
    stack<char> inverseStack;
    initMaze();
    printMaze();
    do{
        showCoordinates();
        cin >> bits;
        done = robotMovement(generalStack, fatherStack, inverseStack, bits);
        printMaze();
        showAllStacks(generalStack, fatherStack, inverseStack);
    }while(!done);
    return 0;
}
