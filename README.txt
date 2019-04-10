
track_generation: contiene la pista ovale, l'update della target position da 
	 	  da raggiungere (per ora prende sempre la metà del sample
		  successivo. va sostituito con quello vero) e chiama la funzione
		  che controlla su la target position è dentro alla pista.

track_constraints: funzione che restituisce 1 o -1 se la target position è dentro
		   la pista o meno. 