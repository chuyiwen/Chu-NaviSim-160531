CFLAGS		= -O -c -g
PROGRAM1 	= main
HDRS1		= car.h define.h dijkstra.h gen_graph.h object.h
OBJS1		= main.o car.o dijkstra.o gen_graph.o 
LIBS1		= -lm



$(PROGRAM1)	:$(OBJS1)
	g++ $(OBJS1) -o $(PROGRAM1) $(LIBS1)

main.o : main.c $(HDRS1)
	g++ $(CFLAGS) main.c

car.o : car.c $(HDRS1)
	g++ $(CFLAGS) car.c

dijkstra.o : dijkstra.c $(HDRS1)
	g++ $(CFLAGS) dijkstra.c

gen_graph.o : gen_graph.o $(HDRS1)
	g++ $(CFLAGS) gen_graph.c

clean:
	rm -f *.o core main
