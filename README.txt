
track_generation: contiene la pista ovale, l'update della target position da 
	 	  da raggiungere (per ora prende sempre la met� del sample
		  successivo. va sostituito con quello vero) e chiama la funzione
		  che controlla su la target position � dentro alla pista.

track_constraints: funzione che restituisce 1 o -1 se la target position � dentro
		   la pista o meno. 