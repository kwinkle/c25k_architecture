import pandas as pd

name="Dave"

end_dict = {
	 0: "Ok bring the treadmill to a stop when you're ready. FINISHED!! You did really well today, definitely looks like I'm going to have to test you next time. Until then " + name + " rest, recover and look after yourself! Goodbye.",
	 1: "Ok " + name + " that's the end of our session for today. Bring the treadmill to a stop when you're ready. I look forward to seeing you again next time!. Goodbye!",
	 2: "Run Complete!! Great work today "+ name + " you are finally free! Bring it down to zero when you're ready. I'll be seeing you soon, keep well! Goodbye.",
	 3: "Ok, bring it down to a stop when you're ready. Nice work " + name + ". We're done for today, but don't forget to cool down properly and have a good stretching session; feel free to talk to my friend Don for some useful stretches. I'll catch you soon, Goodbye.",
	 4: "And we are done!! Catch your breath, cool down and keep up all this great work! I'll see you soon. Goodbye.",
	 5: "Ok, bring it down to a stop when you're ready. Another great session today! Don't forget to have a good stretch out and fuel that body with something worthwhile, you've earnt it. Goodbye.",
	 6: "Done! Bring it down to zero when you're ready. That's all for today's session, I look forward to working you hard again, till next time "+ name + " Goodbye.", 
	 7: "Ok bring it down to zero when you're ready " + name + " that's it for today, you're flying through these runs! I hope you're still having as much fun as I am! I look forward to seeing you next time! Rest well. Goodbye.",
	 8: "Run completed! You survived!! Bring it down to zero when you're ready. I knew you could do it, now rest up and stretch out that body; I think you've earnt some quality relaxation time! Till next time " + name + " Goodbye.",
	 9: "Finished! Bring the treadmill to a stop when you're ready. Nice work today " + name + ". Hope you had a good session. Can't wait until next time! Goodbye.",
	 "Ok bring the treadmill to a stop when you're ready. FINISHED!! You did really well today, definitely looks like I'm going to have to test you next time. Until then " + name + " rest, recover and look after yourself! Goodbye.": 0,
	 "Ok " + name + " that's the end of our session for today. Bring the treadmill to a stop when you're ready. I look forward to seeing you again next time!. Goodbye!": 1,
	 "Run Complete!! Great work today "+ name + " you are finally free! Bring it down to zero when you're ready. I'll be seeing you soon, keep well! Goodbye.": 2,
	 "Ok, bring it down to a stop when you're ready. Nice work " + name + ". We're done for today, but don't forget to cool down properly and have a good stretching session; feel free to talk to my friend Don for some useful stretches. I'll catch you soon, Goodbye.": 3,
	 "And we are done!! Catch your breath, cool down and keep up all this great work! I'll see you soon. Goodbye.": 4,
	 "Ok, bring it down to a stop when you're ready. Another great session today! Don't forget to have a good stretch out and fuel that body with something worthwhile, you've earnt it. Goodbye.": 5,
	 "Done! Bring it down to zero when you're ready. That's all for today's session, I look forward to working you hard again, till next time "+ name + " Goodbye.": 6, 
	 "Ok bring it down to zero when you're ready " + name + " that's it for today, you're flying through these runs! I hope you're still having as much fun as I am! I look forward to seeing you next time! Rest well. Goodbye.": 7,
	 "Run completed! You survived!! Bring it down to zero when you're ready. I knew you could do it, now rest up and stretch out that body; I think you've earnt some quality relaxation time! Till next time " + name + " Goodbye.": 8,
	 "Finished! Bring the treadmill to a stop when you're ready. Nice work today " + name + ". Hope you had a good session. Can't wait until next time! Goodbye.": 9
	}

df = pd.read_csv('behaviour_logger.csv', names=['user_id','session_number','state_id','timestamp','action_id','action', \
			'style','duration','validation','speech'])

#df = pd.read_csv('behaviour_logger.csv')

print df.head()

relevant_df = df.loc[(df['user_id']=="DP") & df['speech'].str.contains("Goodbye")].copy()
relevant_df[df['speech'].str.contains("Goodbye")].head()

if relevant_df.empty:
	print "no prev. examples found"

else:
	print relevant_df.head()

	last_utterance = relevant_df['speech'].iloc[-1]

	print "last utterance was:"
	last_utterance = last_utterance.replace("-",", ")

	print last_utterance

	index = end_dict.get(last_utterance)

	if index: 
		print index

		if index == 9:
			choice = 0
		else:
			choice = index + 1
	else:
		choice = 0

	print "so next utterance will be: "
	print end_dict[choice]