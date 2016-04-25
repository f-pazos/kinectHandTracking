import KinectPV2.*;
import java.util.*;
import java.awt.Point;

KinectPV2 kinect;

void setup(){
  size(1024, 848, P3D);
  
  kinect = new KinectPV2(this);
  //Start up methods go here
  kinect.enableColorImg(true);
  kinect.enableDepthImg(true);
  kinect.enableInfraredImg( true );
  
  kinect.init();
}

void draw() {
  background(0);
  PImage ir = kinect.getInfraredImage();
  PImage irGradient = grads( ir );
  PImage irThreshold = isolateTopRange( irGradient, 0.1, false);
  int[] corners = findCorners( irThreshold, 20, 20, .12 );
  
  PImage[] grads = gxgy( ir, corners );
  PImage gx = grads[0];
  PImage gy = grads[1];
  
  PImage votes = hough( irThreshold, gx, gy, corners );
  PImage restrictVotes = isolateTopRange( votes, 0.70, true );
  
  image( irGradient, 0, 0 );
  image( votes, 0, irThreshold.height);
  image( irThreshold, ir.width, 0);
  image( restrictVotes , ir.width, ir.height);
  
  int xMinBoundary = corners[0];
  int yMinBoundary = corners[1];
  int xMaxBoundary = corners[2];
  int yMaxBoundary = corners[3];
  xMaxBoundary = xMaxBoundary - xMinBoundary;
  yMaxBoundary = yMaxBoundary - yMinBoundary;
  
  noFill();
  stroke(255);
  rect(xMinBoundary, yMinBoundary, xMaxBoundary, yMaxBoundary);
  
  text(frameRate, 50, height - 50);
}

/*****************************************************************************/
int[] fingerTips( PImage votes ){
  
  List<Point> topFive = new LinkedList<Point>();
  
  return new int[]{1};
}
PImage hough(PImage edges, PImage gx, PImage gy, int[] boundaries){
  
  PImage hough = createImage(edges.width, edges.height, RGB);
  
  if( boundaries[0] > 0 && boundaries[1] > 0 && boundaries[2] < edges.width && boundaries[3] < edges.height){
    
    int[][] votes = new int[edges.height][edges.width];
    
    for( int r = boundaries[1]; r < boundaries[3]; r++){
      for( int c = boundaries[0]; c < boundaries[2]; c++){
        votes[r][c] = 0;
      }
    }
    
    for( int r = boundaries[1]; r < boundaries[3]; r++){
      for( int c = boundaries[0]; c < boundaries[2]; c++){
        int index = edges.width*r + c;
        
        if( (int)(edges.pixels[index]) != 0){
          double dy = (int)gx.pixels[index];
          double dx = (int)gy.pixels[index];
          
          double mag = sqrt((float)(dy*dy + dx*dx));
          
          dx = dx / mag;
          dy = dy / mag;
          
          double currR = r;
          double currC = c;
          
          //Head out in one direction, increase votes.
          //Reduce the starting point by 1, to prevent doubling up.
          if( currC > boundaries[0] && currR > boundaries[1] && currC < boundaries[2] && currR < boundaries[3] ){
            votes[r][c] -= 1;
          }
          
          while( currC > boundaries[0] && currR > boundaries[1] && currC < boundaries[2] && currR < boundaries[3] ){
            
            votes[(int)currR][(int)currC] += 1;
            
            double prevC = currC;
            double prevR = currR;
            
            while( (int)prevC == (int)currC && (int)prevR == (int)currR){
              currC += dx;
              currR += dy;
            }
          }
            currR = r;
            currC = c;
            
          //Go back in the other direction.
          while( currC > boundaries[0] && currR > boundaries[1] && currC < boundaries[2] && currR < boundaries[3] ){
            
            votes[(int)currR][(int)currC] += 1;
            
            double prevC = currC;
            double prevR = currR;
            
            while( (int)prevC == (int)currC && (int)prevR == (int)currR){
              currC -= dx;
              currR -= dy;
              
            }  
          }
         
        }
        
      }
    }
  
  
  float maxVote = 0;
  
  for( int r = boundaries[1]; r < boundaries[3]; r++){
    for( int c = boundaries[0]; c < boundaries[2]; c++){
      maxVote = max( maxVote, votes[r][c]);
    }
  }

  text(maxVote, 50, height - 100);
  for( int r = boundaries[1]; r < boundaries[3]; r++){
    for( int c = boundaries[0]; c < boundaries[2]; c++){
      hough.set(c, r, (int)(255.0*votes[r][c]/maxVote) );
    }
  }
}
  return hough;
}


PImage[] gxgy(PImage img, int[] boundaries){
  
  int[][] gxW = new int[3][3];
  int[][] gyW = new int[3][3];
  
  gxW[0][0] = -1;
  gxW[0][1] = 0;
  gxW[0][2] = 1;
  gxW[1][0] = -2;
  gxW[1][1] = 0;
  gxW[1][2] = 2;
  gxW[2][0] = -1;
  gxW[2][1] = 0;
  gxW[2][2] = 1;
  
    
  gyW[0][0] = -1;
  gyW[0][1] = -2;
  gyW[0][2] = -1;
  gyW[1][0] = 0;
  gyW[1][1] = 0;
  gyW[1][2] = 0;
  gyW[2][0] = 1;
  gyW[2][1] = 2;
  gyW[2][2] = 1;
  
  PImage xGradient = createImage(img.width, img.height, RGB);
  PImage yGradient = createImage(img.width, img.height, RGB);
  
  img.loadPixels();
  
  //Boundary: [cMin, rMin, cMax, rMax
  for (int r = boundaries[1]; r < boundaries[3]; r++){
    for (int c = boundaries[0]; c < boundaries[2]; c++){
      
      int gx = 0;
      int gy = 0;
      
      for (int rLocal = r-1; rLocal < r+2; rLocal++){
        for (int cLocal = c-1; cLocal < c+2; cLocal++){
          if ( (rLocal > 0) && (rLocal < img.height) && (cLocal > 0) && (cLocal < img.width)){
            
            int index = img.width*rLocal + cLocal;
            
            int localPixel = img.pixels[index];
            
            gx += gxW[rLocal - (r-1)][cLocal - (c-1)] * (int)localPixel;
            gy += gyW[rLocal - (r-1)][cLocal - (c-1)] * (int)localPixel;
         
          }}}
      xGradient.set(c, r, gx);
      yGradient.set(c, r, gy);
    }
  }
  
  return new PImage[]{xGradient, yGradient};
}
//Take only pixels that are greater than threshold*maxValue.
//returns [cMinBoundary, rMinBoundary, cMaxBoundary, rMaxBoundary]
int[] findCorners( PImage img, int xBoxes, int yBoxes, double threshold){
  
  int[][] votes = new int[yBoxes][xBoxes];
  
  for (int r = 0; r < img.height; r++){
    for (int c = 0; c < img.width; c++){
      
      int index = img.width*r + c;
      
      if( (int)img.pixels[index] != 0 ){ 
        
        int currC = (int)( ((1.0 * c) / img.width ) * xBoxes);
        int currR = (int)( ((1.0 * r) / img.height ) *yBoxes);
        
        votes[currR][currC] += 1;
      
      }
    }
  }
  
  int[][] votesBackup = new int[votes.length][];
  
  for(int i = 0; i < votes.length; i++){
    votesBackup[i] = votes[i].clone();
  }
  
  
  int maxVote = 0;
  int maxR = 0;
  int maxC = 0;
  
  for(int r = 1; r < votes.length-1; r++){
    for(int c = 1; c < votes[0].length-1; c++){
      if( votes[r][c] > maxVote){
        maxVote = votes[r][c];
        maxR = r;
        maxC = c;
      }
    }
  }
  
  int rMaxBoundary = maxR;
  int rMinBoundary = maxR;
  int cMaxBoundary = maxC;
  int cMinBoundary = maxC;
  
  Point maximum = new Point( maxC, maxR );
  
  Queue<Point> queue = new LinkedList<Point>();
  
  queue.add( maximum );
  
  while (!queue.isEmpty() ){
    Point curr = queue.poll();
    int r = (int)curr.getY();
    int c = (int)curr.getX();
    
    rMaxBoundary = max( r, rMaxBoundary );
    rMinBoundary = min( r, rMinBoundary );
    cMaxBoundary = max( c, cMaxBoundary );
    cMinBoundary = min( c, cMinBoundary );
    votes[r][c] = 0;
    
    for (int dr = -1; dr < 2; dr++){
      for (int dc = -1; dc < 2; dc++){
        if ( (r+dr> 1) && (r+dr < (votes.length - 1) ) && (c+dc > 1) && (c+dc < (votes[0].length-1) )){
          if ((int)(255.0*votes[r+dr][c+dc]/maxVote) > maxVote*threshold){
            queue.add( new Point(c+dc, r+dr) );
          }
      }
    }
  }
  }
  
  //Draw votes.
  /*PImage result = createImage( img.width, img.height, RGB);
  for(int r = 0; r < img.height; r++){
    for( int c = 0; c < img.width; c++){
      int currC = (int)( (1.0*c) / img.width * xBoxes);
      int currR = (int)( (1.0*r) / img.height *yBoxes);
      
      int val = int( 255.0*votesBackup[currR][currC] / maxVote );
      result.set(c, r, color(val) );
    }
  }
  
  image(result, img.width, img.height);*/
  
   rMaxBoundary = (int)( (1.0*img.height)/yBoxes*(rMaxBoundary + 1.5))-1;
   rMinBoundary = (int)((1.0*img.height)/yBoxes*(rMinBoundary-0.5));
   cMaxBoundary = (int)( (1.0*img.width)/xBoxes*(cMaxBoundary + 1.5))-1;
   cMinBoundary = (int)((1.0*img.width)/xBoxes*(cMinBoundary-0.5));

  return new int[]{cMinBoundary, rMinBoundary, cMaxBoundary, rMaxBoundary};
}

/*****************************************************/
PImage isolateTopRange( PImage img, double threshold, boolean dispText ){
  
  int maxValue = 0;
  
  for( int r = 0; r < img.height; r++){
    for( int c = 0; c < img.width; c++){
      
      int index = img.width*r + c;
      
      if( (int)(img.pixels[index]) > maxValue ){
        maxValue = (int)img.pixels[index];
      }
    }
  }
  
  if( dispText ){
    text( maxValue, 50, height - 100 );
  }
  PImage result = createImage( img.width, img.height, RGB );
  
  for( int r = 0; r < img.height; r++){
    for( int c = 0; c < img.width; c++){
      
      int index = img.width*r + c;
      
      if( (int)img.pixels[index] > maxValue*threshold){
        result.set(c, r, img.get(c, r) );
      }
    }
  }
  
  return result;
  
}

PImage grads(PImage img){
  
  int[][] gxW = new int[3][3];
  int[][] gyW = new int[3][3];
  
  gxW[0][0] = -1;
  gxW[0][1] = 0;
  gxW[0][2] = 1;
  gxW[1][0] = -2;
  gxW[1][1] = 0;
  gxW[1][2] = 2;
  gxW[2][0] = -1;
  gxW[2][1] = 0;
  gxW[2][2] = 1;
  
  gyW[0][0] = -1;
  gyW[0][1] = -2;
  gyW[0][2] = -1;
  gyW[1][0] = 0;
  gyW[1][1] = 0;
  gyW[1][2] = 0;
  gyW[2][0] = 1;
  gyW[2][1] = 2;
  gyW[2][2] = 1;
  
  PImage gradient = createImage(img.width, img.height, RGB);
  
  img.loadPixels();
  gradient.loadPixels();
  
  
  for (int r = 0; r < img.height; r++){
    for (int c = 0; c < img.width; c++){
      
      int gx = 0;
      int gy = 0;
      
      for (int rLocal = r-1; rLocal < r+2; rLocal++){
        for (int cLocal = c-1; cLocal < c+2; cLocal++){
          if ( (rLocal > 0) && (rLocal < img.height) && (cLocal > 0) && (cLocal < img.width)){
            
            int index = img.width*rLocal + cLocal;
            
            int localPixel = img.pixels[index];
            
            gx += gxW[rLocal - (r-1)][cLocal - (c-1)] * (int)localPixel;
            gy += gyW[rLocal - (r-1)][cLocal - (c-1)] * (int)localPixel;
         
          }}}
          
      int val = int( abs(gx) + abs(gy) );
      
      
      
      gradient.set(c, r, val );
    }
  }
  
  maxVal = 0;
  
  for( int c : gradient.pixels){
    maxVal = max( c, maxVal );
  }
  
  
  return gradient;
}