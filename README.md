Java implementation of ORCA collision-avoidance algorithm. 
==============

License:
--------
The code originated as Java port of [RVO2 Library](http://gamma.cs.unc.edu/RVO2/). Therefore it inherits its license terms, read LICENSE for more information. In particular, the code can be used only for research and non-commercial purposes. 

Authors:
--------
The original port of RVO2Lib has been written by Pavel Janovsky (not reflected in the repository history). The authors subsequent changes are tracked properly.

Maven configuration:
--------
The artifacts are stored in a dedicated publicly-accessible repository. Add this to your pom.xml to tell Maven where it needs to search for the artifacts:

	<repositories>
		<repository>
			<id>atg-repo</id>
			<name>atg-repo</name>
			<url>http://jones.felk.cvut.cz/artifactory/repo</url>
			<snapshots>
				<updatePolicy>always</updatePolicy>
			</snapshots>
			<releases />
		</repository>
	</repositories>   


Then add this snippet to your `<dependencies>` section in pom.xml:

      <dependency>
          <groupId>cz.agents.alite</groupId>
          <artifactId>orca</artifactId>
          <version>1.0-SNAPSHOT</version>
      </dependency>


