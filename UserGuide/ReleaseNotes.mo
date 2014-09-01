within SimpleZoneModel.UserGuide;
package ReleaseNotes
  extends Modelica.Icons.ReleaseNotes;
  model Version_0_1_build1
    extends Modelica.Icons.ReleaseNotes;
  annotation(preferredView = "info",
  Documentation(info= "<html>
<p>
First release of the library.
</p>

This version of the library contains a zone model that is based on Annex C of ISO 13790 - 2008.
There is also a model for distributing the solar radiation data from a TMY3 weather file onto the 
glazed portions of the building envelope.  Two examples are implemented that show the use of these 
models together to get the indoor air temperature of the zone.

</p>

<H4> Limitations </H4>
<p>
There are known limitations in the current build of <a href=\"modelica://SimpleZoneModel.BuildingZones.Facades\">SimpleZoneModel.BuildingZones.Facades</a>,
which currently ignores the long wave radiation to the sky and does not account for absorption of solar energy at opaque surfaces.
</p>
</html>"));
  end Version_0_1_build1;
annotation(preferredView="info",
Documentation(info="<html>
<p>
This section summarizes the changes that have been performed on the ISE Library in
producing the SimpleZoneModel library.
</p>
<ul>
<li>
<a href=\"modelica://SimpleZoneModel.UserGuide.ReleaseNotes.Version_0_1_build1\">Version 0.1 build1</a> (01 SEP 2014)
</li>
</ul>
</html>"));
end ReleaseNotes;
