# ============================================================================
#  TelePatin - Convierte una grabación del ESP32-CAM a video
#
#  Uso (en PowerShell, desde la carpeta del proyecto):
#    .\firmware\Camara_Grabadora\convertir_video.ps1 E:\vuelo_001
#        -> vuelo_001.avi  (INSTANTÁNEO: no recodifica, se ve en VLC y en el
#           Reproductor de Windows)
#
#    .\firmware\Camara_Grabadora\convertir_video.ps1 E:\vuelo_001 -Formato mp4
#        -> vuelo_001.mp4  (unos segundos por minuto; se ve en todo: celular,
#           PowerPoint, WhatsApp; ocupa ~3 veces menos)
#
#  Recortar solo el vuelo (hh:mm:ss desde el inicio de la grabación):
#    .\firmware\Camara_Grabadora\convertir_video.ps1 E:\vuelo_001 -Desde 00:18:30 -Duracion 60
#
#  Los fps se calculan con tiempos.csv (imágenes reales / tiempo real), así el
#  video dura exactamente lo mismo que el vuelo.
#  Requiere ffmpeg (instalado en el PATH del usuario).
# ============================================================================
param(
  [Parameter(Mandatory = $true)][string]$Carpeta,
  [ValidateSet('avi', 'mp4')][string]$Formato = 'avi',
  [string]$Desde,      # inicio del recorte, p. ej. 00:18:30 o 1110 (segundos)
  [string]$Duracion    # duración del recorte en segundos, p. ej. 60
)

if (-not (Test-Path -LiteralPath $Carpeta -PathType Container)) {
  Write-Host "No existe la carpeta '$Carpeta'." -ForegroundColor Red
  Write-Host "Revisa la letra de la microSD en 'Este equipo' (por ejemplo D:\vuelo_001)." -ForegroundColor Yellow
  exit 1
}
$video   = Join-Path $Carpeta "video.mjpeg"
$tiempos = Join-Path $Carpeta "tiempos.csv"
if (-not (Test-Path $video)) { Write-Host "No se encontró $video" -ForegroundColor Red; exit 1 }
# Si la terminal se abrió antes de instalar ffmpeg, buscarlo donde se instaló
$ffLocal = Join-Path $env:LOCALAPPDATA "Programs\ffmpeg\bin"
if (-not (Get-Command ffmpeg -ErrorAction SilentlyContinue) -and (Test-Path "$ffLocal\ffmpeg.exe")) {
  $env:Path = "$ffLocal;$env:Path"
}
if (-not (Get-Command ffmpeg -ErrorAction SilentlyContinue)) {
  Write-Error "No se encuentra ffmpeg. Abre una terminal nueva o instálalo y agrégalo al PATH."
  exit 1
}

# fps reales a partir de los tiempos de cada imagen
$fps = 12
if (Test-Path $tiempos) {
  $filas = Import-Csv $tiempos
  if ($filas.Count -ge 2) {
    $ms = [double]$filas[-1].ms - [double]$filas[0].ms
    if ($ms -gt 0) { $fps = [math]::Round(($filas.Count - 1) / ($ms / 1000), 2) }
    Write-Host ("Imágenes: {0}   Duración: {1:N1} s   fps: {2}" -f $filas.Count, ($ms / 1000), $fps)
  }
}

$recorte = @()
if ($Desde)    { $recorte += @('-ss', $Desde) }
if ($Duracion) { $recorte += @('-t', $Duracion) }

$nombre = Split-Path $Carpeta -Leaf
if ($Desde -or $Duracion) { $nombre += "_recorte" }
$salida = Join-Path $Carpeta "$nombre.$Formato"

if ($Formato -eq 'avi') {
  # Solo envuelve las imágenes JPEG en un AVI, sin recodificarlas: instantáneo
  ffmpeg -y -loglevel error -framerate $fps -f mjpeg -i $video @recorte -c:v copy $salida
} else {
  # MP4 H.264 compatible con todo; "veryfast" es ~2 veces más rápido que el normal
  ffmpeg -y -loglevel error -framerate $fps -f mjpeg -i $video @recorte -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p $salida
}
if ($LASTEXITCODE -eq 0) { Write-Host "Listo: $salida" }
