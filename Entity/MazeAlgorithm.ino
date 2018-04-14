//void initMaze(){
//    for(int row=0; row<mazeSize; row++){
//        for(int column=0; column<mazeSize; column++)
//            maze[row][column] = "000000";
//    }
//}
//
//void printMaze(){
//    for(int row=0; row<mazeSize; row++){
//        for(int column=0; column<mazeSize; column++){
//            Serial.print(" |");
//            Serial.print(maze[row][column]);
//            Serial.print("| ");
//        }
//      Serial.println("");
//    }
//    Serial.println("------------------------------------------");
//}
//
////void printStack(StackArray <char> stack){
////  if(!stack.isEmpty())
////    Serial.println(stack.peek());
////}
////void printAllStacks(StackArray<char> generalStack, StackArray<char> fatherStack, StackArray<char> inverseStack){
////    Serial.print("GeneralStack:");
////    printStack(generalStack);
////    Serial.print("FatherStack:"); 
////    printStack(fatherStack);
////    Serial.print("InverseStack:");
////    printStack(inverseStack);
////    Serial.println("");
////}
//
//int countAvailableMovements(){
//    int moves=0;
//    if(maze[actualRow][actualCol][0]=='0' && comeFrom!='W'){   //Will move West
//        if((comeFrom=='S'||comeFrom=='E') && (maze[actualRow][actualCol-1][4]=='0')) moves++; 
//        else if((comeFrom=='N') && (maze[actualRow][actualCol+1][4]=='0')) moves++; 
//    }
//    if(maze[actualRow][actualCol][1]=='0' && comeFrom!='N'){   //Will move North
//        if(maze[actualRow+1][actualCol][4]=='0')
//            moves++;
//    }
//    if(maze[actualRow][actualCol][2]=='0' && comeFrom!='E'){   //Will move East
//        if((comeFrom=='S'||comeFrom=='E') && (maze[actualRow][actualCol+1][4]=='0')) moves++; 
//        else if((comeFrom=='N') && (maze[actualRow][actualCol-1][4]=='0')) moves++; 
//    }
//    if(maze[actualRow][actualCol][3]=='0' && comeFrom!='S'){   //Will move South
//        if(maze[actualRow-1][actualCol][4]=='0')
//            moves++;
//    }
//    return moves;
//}
//
//void addFatherStack(StackArray<char> &generalStack, StackArray<char> &fatherStack){
//    Serial.println("FATHER-1");
//    generalStack.push('#');
//    fatherStack.push('#');
//    Serial.println("FATHER-2");
//    if(maze[actualRow][actualCol][3]=='0' && comeFrom!='S'){   //Will move South
//        Serial.println("FATHER-S");
//        if(maze[actualRow-1][actualCol][4]=='0')
//            fatherStack.push('S');                   
//    }
//    if(maze[actualRow][actualCol][2]=='0' && comeFrom!='E'){   //Will move East
//        Serial.println("FATHER-E");
//        if((comeFrom=='S'||comeFrom=='E') && (maze[actualRow][actualCol+1][4]=='0')) fatherStack.push('E'); 
//        else if((comeFrom=='N') && (maze[actualRow][actualCol-1][4]=='0')) fatherStack.push('E');        
//      }
//    if(maze[actualRow][actualCol][1]=='0' && comeFrom!='N'){   //Will move North
//        Serial.println("FATHER-N");
//        if(maze[actualRow+1][actualCol][4]=='0')
//            fatherStack.push('N');
//    }
//    if(maze[actualRow][actualCol][0]=='0' && comeFrom!='W'){   //Will move West
//      Serial.println("FATHER-W");
//        if((comeFrom=='S'||comeFrom=='E') && (maze[actualRow][actualCol-1][4]=='0')) fatherStack.push('W');
//        else if((comeFrom=='N') && (maze[actualRow][actualCol+1][4]=='0')) fatherStack.push('W');
//    }
//}
//
//void addGeneralStack(StackArray<char> &generalStack){
//    if(maze[actualRow][actualCol][0]=='0' && comeFrom!='W'){   //Will move West
//        if((comeFrom=='S'||comeFrom=='E') && (maze[actualRow][actualCol-1][4]=='0')) generalStack.push('W');
//        else if((comeFrom=='N') && (maze[actualRow][actualCol+1][4]=='0')) generalStack.push('W');           
//    }
//    else if(maze[actualRow][actualCol][1]=='0' && comeFrom!='N'){   //Will move North
//        if(maze[actualRow+1][actualCol][4]=='0')
//            generalStack.push('N');
//    }
//    else if(maze[actualRow][actualCol][2]=='0' && comeFrom!='E'){   //Will move East
//        if((comeFrom=='S'||comeFrom=='E') && (maze[actualRow][actualCol+1][4]=='0')) generalStack.push('E'); 
//        else if((comeFrom=='N') && (maze[actualRow][actualCol-1][4]=='0')) generalStack.push('E');            
//    }
//    else if(maze[actualRow][actualCol][3]=='0' && comeFrom!='S'){   //Will move South
//        if(maze[actualRow-1][actualCol][4]=='0')
//           generalStack.push('S');
//   }
//}
//
//char inverseDirection(char direction){
//    if(direction == 'W')   //Will move West
//        return 'E';
//    else if(direction == 'N')   //Will move North
//        return 'S';
//    else if(direction == 'E')   //Will move East
//        return 'W';
//    else if(direction == 'S')   //Will move South
//        return 'N';
//}
////void addInverseStack(StackArray<char> generalStack, StackArray<char> &inverseStack){
////    StackArray <char>auxStack;
////    while(generalStack.peek()!='#'){
////        auxStack.push(generalStack.peek());
////        generalStack.pop();
////    }
////    while(!auxStack.isEmpty()){
////        inverseStack.push(inverseDirection(auxStack.peek()));
////        auxStack.pop();
////    }
////}
//
//void move(char direction){
//    Serial.println("MOVING");
//    readPosition(bno, event, mpu, 'B');
//    filtrateDistances(ultraFront, ultraRight, ultraLeft);
//    maze[actualRow][actualCol][4] = '1';
//    if(direction=='W'){   //Will move West        
//        if(comeFrom=='S'){
//          actualCol--;
//          spinPID(bno, event, mpu, -90, false);   
//        }                
//        else if(comeFrom=='E'){
//          actualCol--;
//          oneStep(ultraFront, ultraRight, ultraLeft, stepDistance);
//        }
//        else if(comeFrom=='N'){
//          actualCol++;
//          spinPID(bno, event, mpu, 90, false);
//        }
//        comeFrom = 'E';
//    }
//    else if(direction=='N'){   //Will move North
//        actualRow++;
//        if(comeFrom=='W')
//          spinPID(bno, event, mpu, -90, false);
//        else if(comeFrom=='S')
//          oneStep(ultraFront, ultraRight, ultraLeft, stepDistance);
//        else if(comeFrom=='E')
//          spinPID(bno, event, mpu, 90, false);
//        comeFrom = 'S';
//    }
//    else if(direction=='E'){   //Will move East
//        if(comeFrom=='N'){
//          actualCol--;
//          spinPID(bno, event, mpu, -90, false);
//        }
//        else if(comeFrom=='W'){
//          actualCol++;
//          oneStep(ultraFront, ultraRight, ultraLeft, stepDistance);
//        }
//        else if(comeFrom=='S'){
//          actualCol++;
//          spinPID(bno, event, mpu, 90, false);  
//        }     
//        comeFrom = 'W';
//    }
//    else if(direction=='S'){   //Will move South
//        actualRow--;
//        if(comeFrom=='E')
//          spinPID(bno, event, mpu, -90, false);
//        else if(comeFrom=='N')
//          oneStep(ultraFront, ultraRight, ultraLeft, stepDistance);
//        else if(comeFrom=='W')
//          spinPID(bno, event, mpu, 90, false);     
//        comeFrom = 'N';
//    }
//}
//
//String checkSides(String bits, int bit1, int bit2, int bit3){
//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  if(ultraLeft.side)
//    bits[bit1]='1';
//  if(ultraFront.side)
//    bits[bit2]='1';
//  if(ultraRight.side)
//    bits[bit3]='1';
//  return bits;
//}
//String getBitWithValues(){
//  String bits = "000000";
//  switch(comeFrom){
//    case 'W':
//        return checkSides(bits, 1, 2, 3); //bits of N, E, S
//      break;
//    case 'N':
//        return checkSides(bits, 2, 3, 0); //bits of E, S, W
//      break;
//    case 'E':
//        return checkSides(bits, 3, 0, 1); //bits of S, W, N
//      break;
//    case 'S':
//        return checkSides(bits, 0, 1, 2); //bits of W, N, E
//      break;                  
//  }
//}
//
//bool robotMovement(StackArray<char> &generalStack, StackArray<char> &fatherStack, StackArray<char> &inverseStack, String bits){
//    maze[actualRow][actualCol]=bits;
//    int nMovements = countAvailableMovements();
//    bool isFather=false;
//    Serial.print("Bit: ");
//    Serial.println(bits);
//    Serial.print("Moves: ");
//    Serial.println(nMovements);
////    stop(true);
////    delay(500);
////    writeStringLCD(bits, 0, 0);
////    writeIntLCD(nMovements, 0, 1);
//    Serial.print("Come From: ");
//    Serial.println(comeFrom);    
////    char movements[10] = {'E', 'E', 'N', 'W', 'N', 'E', 'N', 'W', 'W', 'S'};
////    for(int i=0; i<10; i++){
////      Serial.println(comeFrom);
////      move(movements[i]);
////    }
//
////    if(nMovements == 0){
////        if(generalStack.peek()=='#' && fatherStack.peek()=='#'){
////            Serial.println("GATO GATO");
////            generalStack.pop();
////            fatherStack.pop();
////            isFather = true;
////        }
////        if(!fatherStack.isEmpty()){
//////            addInverseStack(generalStack, inverseStack);
////            if(generalStack.peek()=='#'){
////                Serial.println("CANCELA MOVIMIENTO");
////                fatherStack.pop();
////                isFather=true;
////            }
////            if(!isFather){
////                Serial.println("GIRO 180");
////                spinPID(bno, event, mpu, 90, true);
////                spinPID(bno, event, mpu, 90, false);
////            }
////            while(generalStack.peek()!='#'){//Returns to last father
////                move(inverseDirection(generalStack.peek()));
////                printMaze();
////                Serial.println("REGRESA");
////  //                printAllStacks(generalStack, fatherStack, inverseStack);                
////                generalStack.pop();
//////                inverseStack.pop();
////            }
////        }
////        else{
////            Serial.println("SE ACABO");
////            return true;//Algorith DONE
////        }
////    }
////    else{
////        if(nMovements > 1){
////            addFatherStack(generalStack, fatherStack);
////            Serial.println("CREA PADRE");
//////            printAllStacks(generalStack, fatherStack, inverseStack);
////        }
////        if(generalStack.peek()=='#' && fatherStack.peek()!='#'){
////            Serial.println("AGREGA MOVIMIENTO A PADRE");
////            generalStack.push(fatherStack.peek());
////            fatherStack.pop();
////        }
////        else{
////            Serial.println("GENERAL MOVEMENT");
////            addGeneralStack(generalStack);
////        }
//////        printAllStacks(generalStack, fatherStack, inverseStack);
////        move(generalStack.peek());
////    }
//    return false;
//}
//
//void mazeAlgorithm(){
//    char casilla;
//    bool done=false;
//    String bits = getBitWithValues();
//    bits[3]='1';
//    StackArray <char> fatherStack;
//    StackArray <char> generalStack;
//    StackArray <char> inverseStack;
//    initMaze();
//    done = robotMovement(generalStack, fatherStack, inverseStack, bits);
//    do{      
//      filtrateDistances(ultraFront, ultraRight, ultraLeft);
//      printMaze();
//      done = robotMovement(generalStack, fatherStack, inverseStack, getBitWithValues());
//    }while(!done);
//}

